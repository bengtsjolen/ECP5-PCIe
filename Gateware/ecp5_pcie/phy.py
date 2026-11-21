from amaranth import *
from amaranth.build import *
from .serdes import K, D, Ctrl, PCIeScrambler
from .phy_rx import PCIePhyRX
from .phy_tx import PCIePhyTX
from .ltssm import PCIeLTSSM
from .dll_tlp import PCIeDLLTLPTransmitter, PCIeDLLTLPReceiver
from .dllp import PCIeDLLPTransmitter, PCIeDLLPReceiver
from .dll import PCIeDLL
from .virtual_tlp_gen import PCIeVirtualTLPGenerator
from .tlp import TLP
from .packet_align import PCIePacketAligner

class PCIePhy(Elaboratable): # Phy might not be the right name for this
	"""
	A PCIe Phy
	"""
	def __init__(self, lane, upstream = True, support_5GTps = True, disable_scrambling = False, transaction_layer = True):
		self.upstream = upstream
		self.transaction_layer = transaction_layer
		
		# PHY
		self.descrambled_lane = PCIeScrambler(lane)#, Signal())
		self.rx = PCIePhyRX(lane, self.descrambled_lane)
		self.tx = PCIePhyTX(self.descrambled_lane)
		self.ltssm = PCIeLTSSM(self.descrambled_lane, self.tx, self.rx, upstream=upstream, support_5GTps=support_5GTps, disable_scrambling=disable_scrambling) # It doesn't care whether the lane is scrambled or not, since it only uses it for RX detection in Detect
		
		self.packet_aligner = PCIePacketAligner()

		# DLL
		self.dllp_rx = PCIeDLLPReceiver()
		self.dllp_tx = PCIeDLLPTransmitter()

		self.dll = PCIeDLL(self.ltssm, self.dllp_tx, self.dllp_rx, lane.frequency, use_speed = self.descrambled_lane.use_speed)

		self.dll_tlp_rx = (ResetInserter(~self.dll.up))(PCIeDLLTLPReceiver(self.dll))
		self.dll_tlp_tx = (ResetInserter(~self.dll.up))(PCIeDLLTLPTransmitter(self.dll))

		self.debug = Signal(32)
		self.debug2 = Signal(8)

		# TL
		# Without a transaction layer, dll_tlp_rx.tlp_source and dll_tlp_tx.tlp_sink are left
		# for the instantiating module to connect.
		if not self.transaction_layer:
			self.tlp = None

		elif self.upstream:
			self.tlp = TLP()
		
		else:
			self.tlp = PCIeVirtualTLPGenerator()
		
		# Debug
		self.submodules = [
			self.rx,
			self.tx,
			self.ltssm,
			self.dllp_rx,
			self.dllp_tx,
			self.dll,
			self.dll_tlp_rx,
			self.dll_tlp_tx,
		]
		if self.tlp is not None:
			self.submodules.append(self.tlp)

		self.state = [
			self.descrambled_lane.enable,
		]

	def elaborate(self, platform: Platform) -> Module:
		m = Module()

		#m.submodules += [
		#	self.rx,
		#	self.tx,
		#	self.descrambled_lane,
		#	self.ltssm,
		#	self.dllp_rx,
		#	self.dllp_tx,
		#	self.dll,
		#	self.dll_tlp_tx,
		#	self.virt_tlp_gen,
		#]
		m.submodules.rx = self.rx
		m.submodules.tx = self.tx
		m.submodules.descrambled_lane = self.descrambled_lane
		m.submodules.ltssm = self.ltssm
		m.submodules.packet_aligner = self.packet_aligner
		m.submodules.dllp_rx = self.dllp_rx
		m.submodules.dllp_tx = self.dllp_tx
		m.submodules.dll = self.dll
		m.submodules.dll_tlp_tx = self.dll_tlp_tx
		m.submodules.dll_tlp_rx = self.dll_tlp_rx
		if self.tlp is not None:
			m.submodules.tlp = self.tlp

		m.d.comb += self.dll.speed.eq(self.descrambled_lane.speed)

		self.dllp_tx.phy_source.connect(self.tx.sink, m.d.comb)
		self.rx.source.connect(self.packet_aligner.sink, m.d.comb)
		self.packet_aligner.source.connect(self.dllp_rx.phy_sink, m.d.comb)

		self.dll_tlp_tx.dllp_source.connect(self.dllp_tx.dllp_sink, m.d.comb)
		m.d.comb += self.dllp_tx.hold.eq(self.dll_tlp_tx.busy)
		m.d.comb += self.tx.hold_skp.eq(self.dll_tlp_tx.busy | (self.dll_tlp_tx.tlp_sink.valid[0] & self.dll_tlp_tx.tlp_sink.ready))
		self.dllp_rx.dllp_source.connect(self.dll_tlp_rx.dllp_sink, m.d.comb)

		# Return flow control credits as each received TLP leaves the RX buffer. Completions
		# are advertised as infinite and not counted.
		rx_tlp = self.dll_tlp_rx.tlp_source
		rx_tlp_prev = Signal()
		m.d.rx += rx_tlp_prev.eq(rx_tlp.valid[0])
		fmt_type = rx_tlp.symbol[0]
		has_data = fmt_type[6]
		tlp_type = fmt_type[0:5]
		length = Cat(rx_tlp.symbol[3], rx_tlp.symbol[2][0:2])
		data_credits = Signal(12)
		m.d.comb += data_credits.eq(Mux(length == 0, 256, (length + 3) >> 2))
		released = self.dll.credits_released
		with m.If(~self.dll.up):
			m.d.rx += [released.PH.eq(0), released.PD.eq(0), released.NPH.eq(0), released.NPD.eq(0)]
		with m.Elif(rx_tlp.valid[0] & ~rx_tlp_prev):
			with m.If(((tlp_type == 0) & has_data) | (tlp_type[3:5] == 0b10)):
				m.d.rx += released.PH.eq(released.PH + 1)
				with m.If(has_data):
					m.d.rx += released.PD.eq(released.PD + data_credits)
			with m.Elif((tlp_type != 0b01010) & (tlp_type != 0b01011)):
				m.d.rx += released.NPH.eq(released.NPH + 1)
				with m.If(has_data):
					m.d.rx += released.NPD.eq(released.NPD + 1)

		if self.tlp is None:
			pass

		elif self.upstream:
			self.tlp.tlp_source.connect(self.dll_tlp_tx.tlp_sink, m.d.comb)
			self.dll_tlp_rx.tlp_source.connect(self.tlp.tlp_sink, m.d.comb)
		
		else:
			self.tlp.tlp_source.connect(self.dll_tlp_tx.tlp_sink, m.d.comb)
		
		m.d.comb += self.debug.eq(Cat(self.dll_tlp_tx.tlp_sink.symbol))
		m.d.comb += self.debug2.eq(Cat(self.dll_tlp_tx.tlp_sink.valid))

		#m.submodules.dlrx=	self.dllp_rx
		#m.submodules.dltx=	self.dllp_tx
		#m.submodules.dll =	self.dll
		#m.d.comb		+=	self.dllp_tx.enable.eq(self.tx.enable_higher_layers)
		#	self.dllp_tx,


		# TESTING
		
		#m.d.rx += self.dllp_tx.dllp.eq(self.dllp_rx.fifo.r_data)
		#m.d.rx += self.dllp_rx.fifo.r_en.eq(1)
		#m.d.rx += self.tx.fifo.w_en.eq(self.rx.fifo.r_rdy)
		#m.d.rx += self.rx.fifo.r_en.eq(self.rx.fifo.r_rdy)
		#counter = Signal(8)
		#m.d.rx += counter.eq(counter + 1)
		#m.d.rx += self.tx.fifo.w_data.eq(counter)
		#m.d.rx += self.tx.fifo.w_en.eq(counter < 128)




		m.d.rx += self.descrambled_lane.rx_align.eq(1)

		m.d.rx += self.descrambled_lane.enable.eq(self.ltssm.status.link.scrambling & ~self.tx.sending_ts)
		# Received ordered sets are parsed from the raw lane, so RX descrambling must not follow
		# what TX is sending: a SKP sent by TX would leave the word received with it scrambled.
		m.d.rx += self.descrambled_lane.rx_enable.eq(self.ltssm.status.link.scrambling)

		return m