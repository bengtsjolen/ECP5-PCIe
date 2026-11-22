from amaranth import *
from amaranth.build import *
from amaranth.lib.fifo import SyncFIFOBuffered

from .layouts import dllp_layout
from .serdes import K, D, Ctrl
from .crc import LCRC
from .stream import StreamInterface
from .dll import PCIeDLL
from .memory import TLPBuffer

class PCIeDLLTLPTransmitter(Elaboratable):
	"""
	"""
	def __init__(self, dll: PCIeDLL, ratio: int = 4):
		self.tlp_sink = StreamInterface(8, ratio, name="TLP_Sink")
		self.dllp_source = StreamInterface(9, ratio, name="DLLP_Source") # TODO: Maybe connect these in elaborate instead of where this class is instantiated

		self.dll = dll

		self.send = Signal()
		self.started_sending = Signal()
		self.busy = Signal() # A TLP is being framed, from its first symbol until END
		self.accepts_tlps = Signal()
		self.nullify = Signal() # if this is 1 towards the end of the TLP, the TLP will be nullified (set to 1 in rx domain, will be set to 0 by this module)
		assert len(self.dllp_source.symbol) == 4
		assert len(self.tlp_sink.symbol) == 4
		self.ratio = len(self.dllp_source.symbol)

		self.clocks_per_ms = 62500

		# See page 142 in PCIe 1.1
		self.next_transmit_seq = Signal(12, reset=0x000) # TLP sequence number
		self.ackd_seq = Signal(12, reset=0xFFF) # Last acknowledged TLP
		self.replay_num = Signal(2, reset=0b00) # Number of times the retry buffer has been re-transmitted
		self.replay_timeout = 462 # TODO: is 462 right number? See page 148 Table 3-4 PCIe 1.1
		self.replay_timer = Signal(range(self.replay_timeout), reset=0)  # Time since last TLP has finished transmitting, hold if LTSSM in recovery
		self.replay_timer_running = Signal()


		#self.tlp_data = Signal(4 * 8)
		self.debug_crc_output = Signal(32)
		self.debug_crc_input = Signal(32)
		self.debug = Signal(8)
		self.debug_state = Signal(4)

		self.buffer = TLPBuffer(ratio = ratio, max_tlps = 2 ** len(self.replay_num))
		self.unacknowledged_tlp_fifo = SyncFIFOBuffered(width = 12, depth = self.buffer.max_tlps)
		self.replay_state = Signal(2)
		self.replay_started = Signal()
		"""1 for a cycle when a replay starts"""
		self.replay_abandoned = Signal()
		"""1 for a cycle when the retry buffer did not deliver the TLP to replay"""

		self.state = [
			self.debug_state
		]

	def elaborate(self, platform: Platform) -> Module:
		m = Module()

		ratio = self.ratio
		assert ratio == 4

		# Maybe these should be moved into PCIeDLLTLP class since it also involves RX a bit
		m.submodules.buffer = buffer = self.buffer
		m.submodules.unacknowledged_tlp_fifo = unacknowledged_tlp_fifo = DomainRenamer("rx")(self.unacknowledged_tlp_fifo)

		m.d.comb += self.dll.status.retry_buffer_occupation.eq(buffer.slots_occupied)
		m.d.comb += self.dll.status.tx_seq_num.eq(self.next_transmit_seq)

		source_from_buffer = Signal()
		sink_ready = Signal()
		m.d.comb += buffer.tlp_source.ready.eq(sink_ready & source_from_buffer)
		sink_valid = Mux(source_from_buffer, buffer.tlp_source.all_valid, self.tlp_sink.all_valid)
		sink_symbol = [Mux(source_from_buffer, buffer.tlp_source.symbol[i], self.tlp_sink.symbol[i]) for i in range(ratio)]

		# The framing FSM starts on a rising valid and ends on a falling one, and the retry buffer
		# stores in parallel. So a source must wait for ready, then send the whole TLP with valid
		# held, then drop valid. Ready is low during a replay and while a TLP's END is framed.
		for i in range(ratio):
			m.d.comb += buffer.tlp_sink.symbol[i].eq(self.tlp_sink.symbol[i])
			m.d.comb += buffer.tlp_sink.valid[i].eq(self.tlp_sink.valid[i])
		ending = Signal()
		m.d.comb += self.tlp_sink.ready.eq(buffer.tlp_sink.ready & ~source_from_buffer & ~ending)

		with m.If(self.dll.up):
			m.d.comb += buffer.store_tlp.eq(1) # TODO: Is this a good idea?
			m.d.rx += buffer.store_tlp_id.eq(self.next_transmit_seq)
			m.d.rx += buffer.send_tlp_id.eq(unacknowledged_tlp_fifo.r_data)
		
		with m.Else():
			m.d.rx += self.next_transmit_seq.eq(self.next_transmit_seq.reset)
			m.d.rx += self.ackd_seq.eq(self.ackd_seq.reset)
			m.d.rx += self.replay_num.eq(self.replay_num.reset)
		
		m.d.rx += self.accepts_tlps.eq((self.next_transmit_seq - self.ackd_seq) >= 2048) # mod 4096 is already applied since the signal is 12 bits long

		with m.If(self.replay_timer_running):
			m.d.rx += self.replay_timer.eq(self.replay_timer + 1)

		with m.If(unacknowledged_tlp_fifo.r_level == 0):
			m.d.rx += self.replay_timer_running.eq(0)
			m.d.rx += self.replay_timer.eq(0)

		# Replays only the oldest unacknowledged TLP, so sources must keep at most one outstanding
		# (retry_buffer_occupation == 0 before starting the next).
		replay_guard = Signal(5)
		with m.FSM(name="Replay_FSM", domain = "rx") as replay_fsm:
			m.d.comb += self.replay_state.eq(replay_fsm.state)

			with m.State("Idle"):
				m.d.rx += source_from_buffer.eq(0)

				replay = (self.replay_timer >= self.replay_timeout) | (self.dll.received_ack_nak & ~self.dll.received_ack)
				with m.If(replay & ~buffer.slots_empty & ~self.busy & ~self.tlp_sink.all_valid):
					m.d.rx += source_from_buffer.eq(1)
					m.d.comb += self.replay_started.eq(1)
					m.next = "Replay"

			with m.State("Replay"):
				m.d.rx += self.replay_timer.eq(0)
				m.d.rx += replay_guard.eq(0)

				with m.If(buffer.slots_empty):
					m.d.rx += source_from_buffer.eq(0)
					m.next = "Idle"

				with m.Elif(~buffer.sending_tlp):
					m.d.comb += buffer.send_tlp.eq(1)
					m.next = "Replay-Start"

			with m.State("Replay-Start"):
				m.d.rx += self.replay_timer.eq(0)
				m.d.rx += replay_guard.eq(replay_guard + 1)

				with m.If(self.busy):
					m.next = "Replay-End"

				with m.Elif(replay_guard == 31): # The buffer did not find the TLP
					m.d.comb += self.replay_abandoned.eq(1)
					m.d.rx += source_from_buffer.eq(0)
					m.next = "Idle"

			with m.State("Replay-End"):
				m.d.rx += self.replay_timer.eq(0)

				with m.If(~self.busy):
					m.d.rx += source_from_buffer.eq(0)
					m.next = "Idle"
		
		m.d.rx += self.ackd_seq.eq(self.dll.received_ack_nak_id)

		# Ack and Nak both acknowledge every TLP up to their ID, and the partner may acknowledge
		# several TLPs with one Ack. Drop all of them from the retry buffer and the FIFO.
		m.d.comb += buffer.ack_enable.eq(self.dll.up)
		m.d.comb += buffer.ack_id.eq(self.ackd_seq)
		fifo_age = Signal(12)
		m.d.comb += fifo_age.eq(self.ackd_seq - unacknowledged_tlp_fifo.r_data)
		m.d.comb += unacknowledged_tlp_fifo.r_en.eq(unacknowledged_tlp_fifo.r_rdy & ~fifo_age[11])

		with m.If(self.dll.received_ack_nak):
			m.d.comb += buffer.in_buffer_id.eq(self.dll.received_ack_nak_id)
			with m.If(buffer.in_buffer):
				m.d.rx += self.replay_timer.eq(0)
		
		reset_crc = Signal(reset = 1)
		# TODO Warning: Endianness
		crc_input = Signal(32)
		m.submodules.lcrc = lcrc = DomainRenamer("rx")(LCRC(crc_input, reset_crc))
		m.d.rx += self.debug_crc_output.eq(lcrc.output)
		m.d.rx += self.debug_crc_input.eq(crc_input)

		last_valid = Signal()
		m.d.rx += last_valid.eq(sink_valid)
		last_last_valid = Signal() # TODO: Maybe there is a better way
		m.d.rx += last_last_valid.eq(last_valid)
		last_last_last_valid = Signal() # TODO: Maybe there is a better way 😿 like a longer signal which gets shifted for example
		m.d.rx += last_last_last_valid.eq(last_last_valid)


		tlp_bytes = Signal(8 * self.ratio)
		tlp_bytes_before = Signal(8 * self.ratio)

		with m.If(sink_valid):
			m.d.comb += Cat(tlp_bytes[0 : 8 * ratio]).eq(Cat(sink_symbol)) # TODO: Endianness correct?
			m.d.rx += Cat(tlp_bytes_before[0 : 8 * ratio]).eq(Cat(sink_symbol)) # TODO: Endianness correct?

		with m.Else():
			with m.If(self.nullify):
				m.d.comb += Cat(tlp_bytes[0 : 8 * ratio]).eq(~lcrc.output) # ~~x = x
				m.d.rx += Cat(tlp_bytes_before[0 : 8 * ratio]).eq(~lcrc.output)
				m.d.comb += buffer.delete_tlp.eq(1)
				m.d.comb += buffer.delete_tlp_id.eq(self.next_transmit_seq)

			with m.Else():
				m.d.comb += Cat(tlp_bytes[0 : 8 * ratio]).eq(lcrc.output) # TODO: Endianness correct?
				m.d.rx += Cat(tlp_bytes_before[0 : 8 * ratio]).eq(lcrc.output) # TODO: Endianness correct?


		#m.d.rx += Cat(tlp_bytes[8 * ratio : 2 * 8 * ratio]).eq(Cat(tlp_bytes[0 : 8 * ratio]))

		even_more_delay = [Signal(9) for i in range(4)]

		m.d.rx += unacknowledged_tlp_fifo.w_en.eq(0)

		m.d.comb += sink_ready.eq(0) # TODO: maybe move to rx?

		delayed_symbol = Signal(32)
		m.d.rx += delayed_symbol.eq(Cat(sink_symbol))
		m.d.comb += crc_input.eq(delayed_symbol)

		for i in range(4):
			m.d.rx += self.dllp_source.valid[i].eq(0)


		with m.If(self.dllp_source.ready & unacknowledged_tlp_fifo.w_rdy):
			m.d.comb += sink_ready.eq(1) # TODO: maybe move to rx?
			with m.FSM(name = "DLL_TLP_tx_FSM", domain = "rx") as fsm:
				m.d.comb += Cat(self.debug[0:4]).eq(fsm.state)
				m.d.comb += self.debug_state.eq(fsm.state)
				m.d.comb += self.debug[4].eq(reset_crc)

				with m.State("Idle"):
					# A replay keeps the TLP's original sequence number.
					seq = Mux(source_from_buffer, unacknowledged_tlp_fifo.r_data, self.next_transmit_seq)
					with m.If(~last_valid & sink_valid):
						m.d.comb += reset_crc.eq(0)
						m.d.comb += crc_input.eq(Cat(seq[8 : 12], Const(0, shape = 4), seq[0 : 8]))
						m.d.rx += even_more_delay[0].eq(Ctrl.STP)
						m.d.rx += even_more_delay[1].eq(seq[8 : 12])
						m.d.rx += even_more_delay[2].eq(seq[0 : 8])
						m.d.rx += even_more_delay[3].eq(tlp_bytes[8 * 0 : 8 * 1])
						m.next = "Transmit"

				with m.State("Transmit"):
					with m.If(last_valid & sink_valid):
						m.d.comb += reset_crc.eq(0)
						m.d.rx += even_more_delay[0].eq(tlp_bytes_before[8 * 1 : 8 * 2])
						m.d.rx += even_more_delay[1].eq(tlp_bytes_before[8 * 2 : 8 * 3])
						m.d.rx += even_more_delay[2].eq(tlp_bytes_before[8 * 3 : 8 * 4])
						m.d.rx += even_more_delay[3].eq(tlp_bytes[8 * 0 : 8 * 1])
						for i in range(4):
							m.d.rx += self.dllp_source.symbol[i].eq(even_more_delay[i])
						for i in range(4):
							m.d.rx += self.dllp_source.valid[i].eq(1)

					with m.Elif(~sink_valid):
						m.d.comb += reset_crc.eq(0)
						m.d.comb += sink_ready.eq(0) # TODO: maybe move to rx?
						m.d.rx += even_more_delay[0].eq(tlp_bytes_before[8 * 1 : 8 * 2])
						m.d.rx += even_more_delay[1].eq(tlp_bytes_before[8 * 2 : 8 * 3])
						m.d.rx += even_more_delay[2].eq(tlp_bytes_before[8 * 3 : 8 * 4])
						for i in range(4):
							m.d.rx += self.dllp_source.symbol[i].eq(even_more_delay[i])
						for i in range(4):
							m.d.rx += self.dllp_source.valid[i].eq(1)
						m.next = "Post-1"

				with m.State("Post-1"):
					m.d.rx += self.dllp_source.symbol[3].eq(tlp_bytes[8 * 0 : 8 * 1])

					for i in range(3):
						m.d.rx += self.dllp_source.symbol[i].eq(even_more_delay[i])

					for i in range(4):
						m.d.rx += self.dllp_source.valid[i].eq(1)

					m.next = "Post-2"
				
				with m.State("Post-2"):
					m.d.comb += sink_ready.eq(0) # TODO: maybe move to rx?
					m.d.rx += self.dllp_source.symbol[0].eq(tlp_bytes_before[8 * 1 : 8 * 2])
					m.d.rx += self.dllp_source.symbol[1].eq(tlp_bytes_before[8 * 2 : 8 * 3])
					m.d.rx += self.dllp_source.symbol[2].eq(tlp_bytes_before[8 * 3 : 8 * 4])

					with m.If(self.nullify):
						m.d.rx += self.dllp_source.symbol[3].eq(Ctrl.EDB)
						m.d.rx += self.nullify.eq(0)

					with m.Else():
						m.d.rx += self.dllp_source.symbol[3].eq(Ctrl.END)
						with m.If(~source_from_buffer):
							m.d.rx += unacknowledged_tlp_fifo.w_data.eq(self.next_transmit_seq)
							m.d.rx += unacknowledged_tlp_fifo.w_en.eq(1)
							m.d.rx += self.next_transmit_seq.eq(self.next_transmit_seq + 1)
					
					m.d.rx += self.replay_timer_running.eq(1) # TODO: Maybe this should be in the Else block above

					for i in range(4):
						m.d.rx += self.dllp_source.valid[i].eq(1)

					m.next = "Idle"

		m.d.comb += self.busy.eq(~fsm.ongoing("Idle"))
		m.d.comb += ending.eq(fsm.ongoing("Post-1") | fsm.ongoing("Post-2"))


		# TODO: This could be a FSM
		#with m.If(self.dllp_source.ready & unacknowledged_tlp_fifo.w_rdy):
		#	m.d.comb += sink_ready.eq(1) # TODO: maybe move to rx?
#
		#	with m.If(~last_valid & sink_valid):
		#		m.d.comb += reset_crc.eq(0)
		#		m.d.comb += crc_input.eq(Cat(self.next_transmit_seq[8 : 12], Const(0, shape = 4), self.next_transmit_seq[0 : 8]))
		#		m.d.rx += even_more_delay[0].eq(Ctrl.STP)
		#		m.d.rx += even_more_delay[1].eq(self.next_transmit_seq[8 : 12])
		#		m.d.rx += even_more_delay[2].eq(self.next_transmit_seq[0 : 8])
		#		m.d.rx += even_more_delay[3].eq(tlp_bytes[8 * 0 : 8 * 1])
		#		#for i in range(4):
		#		#	m.d.rx += self.dllp_source.symbol[i].eq(even_more_delay[i])
#
		#	with m.Elif(last_valid & sink_valid):
		#		m.d.comb += reset_crc.eq(0)
		#		m.d.rx += even_more_delay[0].eq(tlp_bytes_before[8 * 1 : 8 * 2])
		#		m.d.rx += even_more_delay[1].eq(tlp_bytes_before[8 * 2 : 8 * 3])
		#		m.d.rx += even_more_delay[2].eq(tlp_bytes_before[8 * 3 : 8 * 4])
		#		m.d.rx += even_more_delay[3].eq(tlp_bytes[8 * 0 : 8 * 1])
		#		for i in range(4):
		#			m.d.rx += self.dllp_source.symbol[i].eq(even_more_delay[i])
		#		for i in range(4):
		#			m.d.rx += self.dllp_source.valid[i].eq(1)
#
		#	with m.Elif(last_valid & ~sink_valid): # Maybe this can be done better and replaced by the two if blocks above
		#		m.d.comb += reset_crc.eq(0)
		#		m.d.comb += sink_ready.eq(0) # TODO: maybe move to rx?
		#		m.d.rx += even_more_delay[0].eq(tlp_bytes_before[8 * 1 : 8 * 2])
		#		m.d.rx += even_more_delay[1].eq(tlp_bytes_before[8 * 2 : 8 * 3])
		#		m.d.rx += even_more_delay[2].eq(tlp_bytes_before[8 * 3 : 8 * 4])
		#		for i in range(4):
		#			m.d.rx += self.dllp_source.symbol[i].eq(even_more_delay[i])
		#		for i in range(4):
		#			m.d.rx += self.dllp_source.valid[i].eq(1)
#
		#	with m.Elif(last_last_valid & ~sink_valid):
		#		m.d.rx += self.dllp_source.symbol[3].eq(tlp_bytes[8 * 0 : 8 * 1])
#
		#		for i in range(3):
		#			m.d.rx += self.dllp_source.symbol[i].eq(even_more_delay[i])
#
		#		for i in range(4):
		#			m.d.rx += self.dllp_source.valid[i].eq(1)
#
		#	with m.Elif(last_last_last_valid & ~sink_valid):
		#		m.d.comb += sink_ready.eq(0) # TODO: maybe move to rx?
		#		m.d.rx += self.dllp_source.symbol[0].eq(tlp_bytes_before[8 * 1 : 8 * 2])
		#		m.d.rx += self.dllp_source.symbol[1].eq(tlp_bytes_before[8 * 2 : 8 * 3])
		#		m.d.rx += self.dllp_source.symbol[2].eq(tlp_bytes_before[8 * 3 : 8 * 4])
#
		#		with m.If(self.nullify):
		#			m.d.rx += self.dllp_source.symbol[3].eq(Ctrl.EDB)
		#			m.d.rx += self.nullify.eq(0)
#
		#		with m.Else():
		#			m.d.rx += self.dllp_source.symbol[3].eq(Ctrl.END)
		#			m.d.rx += unacknowledged_tlp_fifo.w_data.eq(self.next_transmit_seq)
		#			m.d.rx += unacknowledged_tlp_fifo.w_en.eq(1)
		#			m.d.rx += self.next_transmit_seq.eq(self.next_transmit_seq + 1)
		#		
		#		m.d.rx += self.replay_timer_running.eq(1) # TODO: Maybe this should be in the Else block above
#
		#		for i in range(4):
		#			m.d.rx += self.dllp_source.valid[i].eq(1)


		return m

class PCIeDLLTLPReceiver(Elaboratable):
	"""
	"""
	def __init__(self, dll: PCIeDLL, ratio: int = 4):
		#self.tlp_source = StreamInterface(8, ratio, name="TLP_Source")
		self.dllp_sink = StreamInterface(9, ratio, name="DLLP_Sink") # TODO: Maybe connect these in elaborate instead of where this class is instantiated

		# Buffer
		self.buffer = TLPBuffer(ratio = ratio, max_tlps = 4, drop_unstorable = True)
		self.tlp_source = self.buffer.tlp_source

		self.dll = dll
		
		assert len(self.dllp_sink.symbol) == 4
		assert len(self.tlp_source.symbol) == 4
		self.ratio = len(self.dllp_sink.symbol)

		self.clocks_per_ms = 62500

		# See page 142 in PCIe 1.1
		self.next_receive_seq = Signal(12, reset=0x000) # Expected TLP sequence number
		self.nak_scheduled = Signal(reset = 0)
		self.ack_nak_latency_limit = 154
		self.ack_nak_latency_timer = Signal(range(self.ack_nak_latency_limit), reset=0) # Time since an Ack or Nak DLLP was scheduled for transmission

		self.actual_receive_seq = Signal(12, reset=0x000) # Received TLP sequence number

		self.debug = Signal(8)
		self.debug2 = Signal(32)
		self.debug3 = Signal(32)
		self.debug_state = Signal(8)

		#self.tlp_data = Signal(4 * 8)

		self.state = [
			self.actual_receive_seq,
			self.debug_state
		]

	def elaborate(self, platform: Platform) -> Module:
		m = Module()

		ratio = self.ratio
		assert ratio == 4

		# TODO: Send NAK if buffer is full
		m.submodules.buffer = buffer = self.buffer
		m.submodules.received_tlp_fifo = received_tlp_fifo = DomainRenamer("rx")(SyncFIFOBuffered(width = 12, depth = buffer.max_tlps))

		m.d.comb += self.dll.status.receive_buffer_occupation.eq(buffer.slots_occupied)
		m.d.comb += self.dll.status.rx_seq_num.eq(self.actual_receive_seq)

		with m.If(~self.dll.up):
			m.d.rx += self.ack_nak_latency_timer.eq(self.ack_nak_latency_timer.reset)
		
		#m.d.rx += self.accepts_tlps.eq((self.next_transmit_seq - self.ackd_seq) >= 2048) # mod 4096 is already applied since the signal is 12 bits long

		#with m.If(self.timer_running):
		#	m.d.rx += self.replay_timer.eq(self.replay_timer + 1)

		source_3_delayed = Signal(8)
		m.d.rx += source_3_delayed.eq(self.dllp_sink.symbol[3][0:8])

		# Aligned as received by TLP layer from other end of the link
		source_symbols = [source_3_delayed, self.dllp_sink.symbol[0][0:8], self.dllp_sink.symbol[1][0:8], self.dllp_sink.symbol[2][0:8]]
		last_symbols = Signal(32)
		m.d.rx += last_symbols.eq(Cat(source_symbols))


		reset_crc = Signal(reset = 1)
		# TODO Warning: Endianness
		crc_input = Signal(32)
		start_crc = Signal()
		m.submodules.lcrc = lcrc = DomainRenamer("rx")(LCRC(crc_input, reset_crc, start_crc))
		m.d.rx += start_crc.eq(0)

		m.d.rx += crc_input.eq(Cat(source_symbols))

		for i in range(4):
			m.d.rx += buffer.tlp_sink.symbol[i].eq(Cat(source_symbols[i]))
		
		end_good = Signal()

		def ack():
			m.d.comb += self.dll.schedule_ack_nak.eq(1)
			m.d.rx += [
				self.dll.scheduled_ack.eq(1),
				self.dll.scheduled_ack_nak_id.eq(self.actual_receive_seq),
				self.ack_nak_latency_timer.eq(self.ack_nak_latency_timer.reset),
			]
		
		def nak():
			with m.If(~self.nak_scheduled):
				m.d.comb += self.dll.schedule_ack_nak.eq(1)
				m.d.rx += [
					self.dll.scheduled_ack.eq(0),
					self.dll.scheduled_ack_nak_id.eq(self.next_receive_seq - 1),
					self.nak_scheduled.eq(1),
					self.ack_nak_latency_timer.eq(self.ack_nak_latency_timer.reset),
				]

		with m.If(self.ack_nak_latency_timer < self.ack_nak_latency_limit):
			m.d.rx += self.ack_nak_latency_timer.eq(self.ack_nak_latency_timer + 1)

		m.d.rx += buffer.delete_tlp.eq(0)

		m.d.comb += self.debug2.eq(lcrc.output)
		m.d.comb += self.debug3.eq(crc_input)

		with m.FSM(name = "DLL_TLP_rx_FSM", domain = "rx") as fsm:
			m.d.comb += Cat(self.debug[0:4]).eq(fsm.state)
			m.d.comb += Cat(self.debug_state[0:4]).eq(fsm.state)

			def start_tlp():
				with m.If(self.dllp_sink.symbol[0] == Ctrl.STP):
					tlp_id = Cat(source_symbols[3], source_symbols[2][0:4])

					m.d.comb += buffer.store_tlp.eq(1)
					m.d.rx += buffer.store_tlp_id.eq(tlp_id)

					m.d.rx += reset_crc.eq(0)
					m.d.rx += start_crc.eq(1)
					m.d.rx += crc_input.eq(Cat(source_symbols[2], source_symbols[3]))
					m.d.rx += self.actual_receive_seq.eq(tlp_id)
					m.next = "Receive"

			with m.State("Idle"):
				start_tlp()
				
				#with m.If(self.dll.up  & (self.ack_nak_latency_timer == self.ack_nak_latency_limit) & ~ self.nak_scheduled)
			
			with m.State("Receive"):
				with m.If((self.dllp_sink.symbol[3] == Ctrl.END) | (self.dllp_sink.symbol[3] == Ctrl.EDB)):
					for i in range(4):
						m.d.rx += buffer.tlp_sink.valid[i].eq(0)

					m.d.rx += end_good.eq(self.dllp_sink.symbol[3] == Ctrl.END)

					m.next = "LCRC"

				with m.Else():
					for i in range(4):
						m.d.rx += buffer.tlp_sink.valid[i].eq(1)
			
			with m.State("LCRC"):
				m.d.rx += reset_crc.eq(1)
				m.d.comb += Cat(self.debug[4:8]).eq(7)

				# Only a TLP the buffer stored may be acknowledged; one it had to drop (no free
				# slot) is ignored and comes back as a replay. A stored copy of anything that
				# isn't the next TLP in sequence is deleted.
				def delete_stored():
					with m.If(buffer.store_ok):
						m.d.rx += buffer.delete_tlp.eq(1)
						m.d.rx += buffer.delete_tlp_id.eq(self.actual_receive_seq)

				with m.If((lcrc.output == last_symbols) & end_good):
					with m.If(self.actual_receive_seq == self.next_receive_seq):
						with m.If(buffer.store_ok):
							m.d.comb += received_tlp_fifo.w_en.eq(1)
							m.d.comb += received_tlp_fifo.w_data.eq(self.actual_receive_seq)
							m.d.rx += self.next_receive_seq.eq(self.next_receive_seq + 1)
							m.d.rx += self.nak_scheduled.eq(0)
							ack()
							m.d.comb += Cat(self.debug[4:8]).eq(1)
						with m.Else():
							m.d.comb += Cat(self.debug[4:8]).eq(2)

					with m.Elif((self.next_receive_seq - self.actual_receive_seq) <= 2048): # Duplicate received
						delete_stored()
						ack()
						m.d.comb += Cat(self.debug[4:8]).eq(3)

					with m.Else():
						delete_stored()
						nak()
						m.d.comb += Cat(self.debug[4:8]).eq(4)

				with m.Else():
					delete_stored()
					with m.If((~lcrc.output == last_symbols) & ~end_good):
						m.d.comb += Cat(self.debug[4:8]).eq(5)
					with m.Else():
						nak()
						m.d.comb += Cat(self.debug[4:8]).eq(6)

				m.next = "Idle"
				# The next TLP can start right after END
				start_tlp()

		# Deliver TLPs in the order they were accepted
		with m.FSM(name = "DLL_TLP_to_TLP_rx_FSM", domain = "rx") as fsm:
			m.d.comb += Cat(self.debug_state[4:8]).eq(fsm.state)

			with m.State("Wait"):
				with m.If(received_tlp_fifo.r_rdy):
					m.d.rx += self.buffer.send_tlp_id.eq(received_tlp_fifo.r_data)
					m.next = "Start"

			with m.State("Start"):
				m.d.comb += self.buffer.send_tlp.eq(1)
				m.next = "Send"

			with m.State("Send"):
				with m.If(~self.buffer.sending_tlp):
					m.d.comb += self.buffer.delete2_tlp.eq(1)
					m.d.comb += self.buffer.delete2_tlp_id.eq(self.buffer.send_tlp_id)
					m.d.comb += received_tlp_fifo.r_en.eq(1)
					m.next = "Wait-Delete"

			with m.State("Wait-Delete"):
				m.next = "Wait"

		return m