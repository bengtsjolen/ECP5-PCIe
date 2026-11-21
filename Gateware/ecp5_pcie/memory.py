from amaranth import *
from amaranth.build import *
import math

from .stream import StreamInterface

class TLPBuffer(Elaboratable):
	"""
	Stores TLPs. They need to be at least 2 clock cycles long. At a ratio of 4 that should be no problem since all TLPs are at least 3 DW long (1 DW = 4 Bytes / Symbols)
	
	Parameters
	----------
	ratio : int
		Gearbox ratio.

	max_tlps : int
		Maximum number of TLPs to store

	tlp_bytes : int
		Maximum number of bytes in a TLP

	delete_on_send : bool
		Whether to delete the TLP once it is sent
	"""
	def __init__(self, ratio: int = 4, max_tlps: int = 4, tlp_bytes: int = 512, delete_on_send: bool = False, drop_unstorable: bool = False):
		self.ratio = ratio
		self.drop_unstorable = drop_unstorable
		"""On store_tlp without a free slot (or with the ID already stored), skip that TLP instead of retrying (receive side)"""
		self.store_ok = Signal()
		"""Whether the TLP of the last store_tlp is being/was stored"""

		self.tlp_sink = StreamInterface(8, ratio, name="TLP_Sink")
		"""Connect this to the TLP source, when all_valid goes low it marks the end of a TLP"""
		self.tlp_source = StreamInterface(8, ratio, name="TLP_Source")
		"""Connect this to the TLP sink"""

		self.max_tlps = max_tlps
		"""Number of TLPs to maximally store"""
		self.tlp_bit_depth = math.ceil(math.log2((tlp_bytes + ratio - 1) // ratio))
		"""log2(TLP memory depth)"""
		self.tlp_depth = 2 ** self.tlp_bit_depth
		"""TLP memory depth"""
		
		self.slots = [[Signal(name=f"Slot_{i}_valid"), Signal(12, name=f"Slot_{i}_ID")] for i in range(max_tlps)] # First signal indicates whether the slot is full
		"""TLP slots, this is a pointer table, first element is whether the pointer is valid and second element is the TLP ID, this is managed by this class, should not be set externally"""

		self.send_tlp_id = Signal(12)
		"""The ID of the TLP to be sent, can be set 1 cycle later than send_tlp"""
		self.send_tlp = Signal()
		"""Set to 1 for 1 cycle to start sending a TLP"""
		self.sending_tlp = Signal()
		"""Is 1 while TLP is being sent"""

		self.delete_tlp_id = Signal(12)
		"""The ID of the TLP to be deleted, should be set in the same cycle as delete_tlp"""
		self.delete_tlp = Signal()
		"""Set to 1 for 1 cycle to delete TLP"""
		self.delete2_tlp = Signal()
		self.delete2_tlp_id = Signal(12)
		"""Second delete port, so two users can't lose each other's deletes"""

		self.store_tlp_id = Signal(12)
		"""The ID of the TLP to be stored, can be set 1 cycle later than store_tlp"""
		self.store_tlp = Signal()
		"""Set to 1 for 1 cycle to start storing a TLP"""
		self.storing_tlp = Signal()
		"""Is 1 while TLP is being stored"""

		self.slots_full = Signal(reset = 0)
		"""Whether all TLP slots are full, check for free space with ~slots_full"""
		self.slots_empty = Signal(reset = 0)
		"""Whether no TLPs are stored"""
		self.slots_occupied = Signal(range(max_tlps + 1))
		"""How many TLPs are stored"""

		self.delete_on_send = delete_on_send
		"""Whether TLPs are deleted after they have been sent"""

		self.ack_enable = Signal()
		"""Delete every TLP whose ID is ack_id or older, modulo 4096 (Ack/Nak are cumulative)"""
		self.ack_id = Signal(12)
		"""Newest acknowledged ID for ack_enable"""

		self.in_buffer = Signal()
		"""Whether TLP in_buffer_id is in the buffer, comb domain"""
		self.in_buffer_id = Signal(12)
		"""ID for in_buffer"""

	def elaborate(self, platform: Platform) -> Module:
		m = Module()

		storage = Memory(width = self.ratio * 8 + 1, depth = self.tlp_depth * self.max_tlps)

		read_port  = m.submodules.read_port  = storage.read_port(domain = "rx", transparent = False)
		write_port = m.submodules.write_port = storage.write_port(domain = "rx")

		slots_empty_statement = 1
		for i in range(self.max_tlps):
			slots_empty_statement = slots_empty_statement & ~self.slots[i][0]

		m.d.comb += self.slots_empty.eq(slots_empty_statement)

		slots_full_statement = 1
		for i in range(self.max_tlps):
			slots_full_statement = slots_full_statement & self.slots[i][0]

		m.d.comb += self.slots_full.eq(slots_full_statement) # TODO: Maybe replace with Cat([i[0] for i in self.slots]).all()? (~….any() for slots_empty)

		read_address_base = Signal(range(self.max_tlps))
		read_address_counter = Signal(range(self.tlp_depth))
		m.d.comb += read_port.addr.eq(Cat(read_address_counter, read_address_base))
		m.d.rx += Cat(self.tlp_source.symbol).eq(read_port.data[:-1])

		tlp_source_valid = Signal(2)
		m.d.rx += tlp_source_valid.eq(tlp_source_valid << 1)
		m.d.rx += tlp_source_valid[0].eq(0)
		m.d.rx += [self.tlp_source.valid[i].eq(tlp_source_valid[-1]) for i in range(self.ratio)]
		m.d.rx += read_port.en.eq(1)


		valid_id_check = 0

		for i in range(self.max_tlps):
			valid_id_check = valid_id_check | self.slots[i][0] & (self.slots[i][1] == self.in_buffer_id)
		
		m.d.comb += self.in_buffer.eq(valid_id_check)

		# Counted from the slots: a store and a delete in the same cycle would lose one update.
		m.d.comb += self.slots_occupied.eq(sum(slot[0] for slot in self.slots))

		with m.FSM(name = "send_fsm", domain = "rx"):
			with m.State("Idle"):
				m.d.rx += tlp_source_valid[0].eq(0)
				m.d.rx += tlp_source_valid[1].eq(0)
				with m.If(self.send_tlp):
					m.d.rx += self.sending_tlp.eq(1)
					m.next = "Set offset"
					
				with m.Else():
					m.d.rx += self.sending_tlp.eq(0)

			with m.State("Set offset"):
				offset = 0
				valid_id = 0

				for i in range(self.max_tlps):
					offset = Mux(self.slots[i][0] & (self.slots[i][1] == self.send_tlp_id), i, offset)
					valid_id = valid_id | self.slots[i][0] & (self.slots[i][1] == self.send_tlp_id)

				m.d.rx += read_address_base.eq(offset)

				with m.If(valid_id & self.tlp_source.ready):
					m.next = "Transmit"
					
				with m.Elif(~valid_id):
					m.d.rx += self.sending_tlp.eq(0)
					m.next = "Idle"
			
			with m.State("Transmit"):
				m.d.rx += tlp_source_valid[0].eq(1)
				with m.If(tlp_source_valid[0]):
					m.d.rx += read_address_counter.eq(read_address_counter + 1)

				with m.If((read_address_counter == self.tlp_depth - 1) | read_port.data[-1]):
					m.d.rx += read_address_counter.eq(0)
					m.d.rx += tlp_source_valid[0].eq(~read_port.data[-1])
					m.d.rx += tlp_source_valid[1].eq(~read_port.data[-1])

					m.d.rx += self.sending_tlp.eq(0)
					m.next = "Idle"

					if self.delete_on_send:
						m.d.comb += self.delete_tlp.eq(1)
						m.d.comb += self.delete_tlp_id.eq(self.send_tlp_id)
		

		# Dereference pointer
		for delete, delete_id in ((self.delete_tlp, self.delete_tlp_id), (self.delete2_tlp, self.delete2_tlp_id)):
			with m.If(delete):
				for i in range(self.max_tlps):
					with m.If(self.slots[i][0] & (delete_id == self.slots[i][1])):
						m.d.rx += self.slots[i][0].eq(0)

		with m.If(self.ack_enable):
			for i in range(self.max_tlps):
				age = Signal(12, name=f"ack_age_{i}")
				m.d.comb += age.eq(self.ack_id - self.slots[i][1])
				with m.If(self.slots[i][0] & ~age[11]):
					m.d.rx += self.slots[i][0].eq(0)


		write_address_base = Signal(range(self.max_tlps))
		write_address_counter = Signal(range(self.tlp_depth), reset = 0)
		write_address = Signal(len(write_port.addr))
		m.d.comb += write_port.addr.eq(write_address)

		m.d.rx += self.tlp_sink.ready.eq(0)
		m.d.rx += write_port.en.eq(0)

		tlp_sink_last_valid = Signal()
		m.d.rx += tlp_sink_last_valid.eq(self.tlp_sink.all_valid)

		# A TLP ends with the first cycle without valid data. The store FSM is back in Idle the
		# cycle after that, so a TLP whose store_tlp comes right after the previous TLP is kept.
		with m.FSM(name = "store_fsm", domain = "rx"):
			offset = 0
			slot_exists = 0
			for i in range(self.max_tlps):
				slot_exists = slot_exists | ((self.slots[i][1] == self.store_tlp_id) & self.slots[i][0])
				offset = Mux(~self.slots[i][0], i, offset)

			def start_store():
				m.d.rx += self.storing_tlp.eq(1)
				m.next = "Set offset"

			with m.State("Idle"):
				with m.If(self.store_tlp & (self.drop_unstorable | ~self.slots_full)):
					start_store()
				with m.Else():
					m.d.rx += self.storing_tlp.eq(0)

			with m.State("Set offset"):
				with m.If(slot_exists | self.slots_full):
					m.d.rx += self.store_ok.eq(0)
					m.next = "Skip" if self.drop_unstorable else "Idle"
				with m.Else():
					m.d.rx += write_address_base.eq(offset)
					m.d.rx += write_address_counter.eq(0)
					m.d.rx += self.store_ok.eq(1)
					m.d.rx += self.tlp_sink.ready.eq(1)
					m.next = "Receive"

			with m.State("Skip"):
				# Let the TLP that can't be stored pass
				with m.If(~self.tlp_sink.all_valid & tlp_sink_last_valid):
					m.next = "Idle"
					with m.If(self.store_tlp):
						start_store()

			with m.State("Receive"):
				m.d.rx += self.tlp_sink.ready.eq(1)

				with m.If(self.tlp_sink.all_valid):
					m.d.rx += write_port.data.eq(Cat(Cat(self.tlp_sink.symbol), 0))
					m.d.rx += write_address.eq(Cat(write_address_counter, write_address_base))
					m.d.rx += write_port.en.eq(1)
					with m.If(write_address_counter != self.tlp_depth - 1):
						m.d.rx += write_address_counter.eq(write_address_counter + 1)

				with m.Elif(tlp_sink_last_valid):
					# Rewrite the last word with the end flag and mark the slot
					m.d.rx += write_port.data.eq(Cat(write_port.data[:-1], 1))
					m.d.rx += write_port.en.eq(1)
					m.d.rx += self.tlp_sink.ready.eq(0)
					for i in range(self.max_tlps):
						with m.If(write_address_base == i):
							m.d.rx += self.slots[i][0].eq(1)
							m.d.rx += self.slots[i][1].eq(self.store_tlp_id)
					m.next = "Idle"
					with m.If(self.store_tlp & (self.drop_unstorable | ~self.slots_full)):
						start_store()

		return m