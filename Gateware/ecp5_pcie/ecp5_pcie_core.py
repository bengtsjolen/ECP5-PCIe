from amaranth import *
from amaranth.build import *
from amaranth.lib.cdc import FFSynchronizer

from .ecp5_serdes_geared_x4 import LatticeECP5PCIeSERDESx4
from .serdes import PCIeSERDESAligner, LinkSpeed, Ctrl
from .phy import PCIePhy
from .ltssm import State

__all__ = ["ECP5PCIeCore"]


class ECP5PCIeCore(Elaboratable):
	"""
	x1 endpoint core for instantiation from LiteX: DCU, PHY, LTSSM and DLL, optionally with the
	ECP5-PCIe transaction layer. Elaborated to Verilog with a flat port list.

	The ``sync`` domain (ports ``clk``/``rst``) is a free-running 100 MHz clock for SCI access;
	with ``fabric_clk`` it is also the DCU reference clock. Otherwise the DCU uses EXTREF0.

	All status and stream ports are in the ``rx`` domain, whose clock is the ``rx_clk_out``
	output. ``tx_clk_out`` is frequency-locked to it through the CTC FIFO.

	Parameters
	----------
	gen : int
		1 for 2.5 GT/s, 2 for 5 GT/s.
	refclk_freq : float
		DCU reference clock in Hz: 100 MHz or 156.25 MHz for Gen1 (default 100 MHz), 200 MHz for
		Gen2.
	fabric_clk : bool
		Use the ``sync`` clock as DCU reference clock instead of EXTREF0.
	transaction_layer : bool
		Include the ECP5-PCIe config-space responder. The ``tlp_*`` ports then only observe the
		DLL streams; ``tlp_rx_ready`` is ignored and ``tlp_tx_ready`` is 0.
	"""
	def __init__(self, gen=1, refclk_freq=None, fabric_clk=False, transaction_layer=False):
		assert gen in (1, 2)
		if refclk_freq is None:
			refclk_freq = 100e6 if gen == 1 else 200e6
		assert refclk_freq in ((100e6, 156.25e6) if gen == 1 else (200e6,))
		assert not (fabric_clk and refclk_freq != 100e6)
		self.gen = gen
		self.transaction_layer = transaction_layer

		self.serdes  = LatticeECP5PCIeSERDESx4(speed_5GTps=(gen == 2), clkfreq=refclk_freq, fabric_clk=fabric_clk)
		self.aligner = DomainRenamer("rx")(PCIeSERDESAligner(self.serdes.lane))
		self.phy     = PCIePhy(self.aligner, support_5GTps=(gen == 2), transaction_layer=transaction_layer)

		self.perst_n = Signal(name="perst_n")

		self.rx_clk_o = Signal(name="rx_clk_out")
		self.tx_clk_o = Signal(name="tx_clk_out")

		self.ltssm_state = Signal(8, name="ltssm_state")
		self.link_up     = Signal(name="link_up")
		self.dll_state   = Signal(8, name="dll_state")
		self.dll_up      = Signal(name="dll_up")

		self.rx_present  = Signal(name="rx_present")
		self.rx_locked   = Signal(name="rx_locked")
		self.rx_aligned  = Signal(name="rx_aligned")
		self.tx_locked   = Signal(name="tx_locked")
		self.reset_done  = Signal(name="reset_done")
		self.det_status  = Signal(name="det_status")

		self.rx_lol_err  = Signal(name="rx_lol_err")
		self.rx_sym_err  = Signal(name="rx_sym_err")

		self.rx_symbol   = Signal(36, name="rx_symbol")
		self.rx_valid    = Signal(4,  name="rx_valid")
		self.tx_symbol   = Signal(36, name="tx_symbol")

		self.credits_rx  = {f: Signal(len(getattr(self.phy.dll.credits_rx, f)), name=f"fc_{f.lower()}")
			for f in ("PH", "PD", "NPH", "NPD", "CPLH", "CPLD")}
		self.retry_buffer_occupation   = Signal(8,  name="retry_buffer_occupation")
		self.receive_buffer_occupation = Signal(8,  name="receive_buffer_occupation")
		self.tx_seq_num  = Signal(12, name="tx_seq_num")
		self.rx_seq_num  = Signal(12, name="rx_seq_num")

		self.tlp_rx_data  = Signal(32, name="tlp_rx_data")
		self.tlp_rx_valid = Signal(4,  name="tlp_rx_valid")
		self.tlp_rx_ready = Signal(name="tlp_rx_ready")
		self.tlp_tx_data  = Signal(32, name="tlp_tx_data")
		self.tlp_tx_valid = Signal(4,  name="tlp_tx_valid")
		self.tlp_tx_ready = Signal(name="tlp_tx_ready")

	def ports(self):
		return [
			self.perst_n, self.rx_clk_o, self.tx_clk_o,
			self.ltssm_state, self.link_up, self.dll_state, self.dll_up,
			self.rx_present, self.rx_locked, self.rx_aligned, self.tx_locked, self.reset_done, self.det_status,
			self.rx_lol_err, self.rx_sym_err,
			self.rx_symbol, self.rx_valid, self.tx_symbol,
			*self.credits_rx.values(),
			self.retry_buffer_occupation, self.receive_buffer_occupation, self.tx_seq_num, self.rx_seq_num,
			self.tlp_rx_data, self.tlp_rx_valid, self.tlp_rx_ready,
			self.tlp_tx_data, self.tlp_tx_valid, self.tlp_tx_ready,
		]

	def elaborate(self, platform: Platform) -> Module:
		m = Module()

		m.submodules.serdes  = serdes  = self.serdes
		m.submodules.aligner = self.aligner
		m.submodules.phy     = phy     = self.phy

		ltssm = phy.ltssm
		lane  = phy.descrambled_lane

		m.d.comb += serdes.lane.speed.eq(LinkSpeed.S2_5)

		m.domains.rx = ClockDomain()
		m.domains.tx = ClockDomain()
		m.d.comb += [
			ClockSignal("rx").eq(serdes.rx_clk),
			ClockSignal("tx").eq(serdes.tx_clk),
			self.rx_clk_o.eq(serdes.rx_clk),
			self.tx_clk_o.eq(serdes.tx_clk),
		]

		# PERST# holds the LTSSM, DLL and the DCU reset sequencer (all in rx) in reset. The DCU
		# keeps producing rx/tx clocks while its resets are asserted.
		perst_rx = Signal(reset=1)
		perst_tx = Signal(reset=1)
		m.submodules += [
			FFSynchronizer(~self.perst_n, perst_rx, o_domain="rx", reset=1),
			FFSynchronizer(~self.perst_n, perst_tx, o_domain="tx", reset=1),
		]

		last_state = Signal(8)
		m.d.rx += last_state.eq(ltssm.debug_state)
		link_lost = (last_state == State.L0) & (ltssm.debug_state == State.Detect)
		m.d.comb += [
			ResetSignal("rx").eq(perst_rx | link_lost),
			ResetSignal("tx").eq(perst_tx),
		]

		training = ltssm.debug_state > State.Polling_Active
		m.d.comb += [
			self.ltssm_state.eq(ltssm.debug_state),
			self.link_up    .eq(ltssm.status.link.up),
			self.dll_state  .eq(phy.dll.debug_state),
			self.dll_up     .eq(phy.dll.up),

			self.rx_present .eq(serdes.lane.rx_present),
			self.rx_locked  .eq(serdes.lane.rx_locked),
			self.rx_aligned .eq(serdes.lane.rx_aligned),
			self.tx_locked  .eq(serdes.lane.tx_locked),
			self.reset_done .eq(serdes.lane.reset_done),
			self.det_status .eq(serdes.lane.det_status),

			self.rx_lol_err .eq(training & ~serdes.lane.rx_locked),
			self.rx_sym_err .eq(training & serdes.lane.has_symbol(Ctrl.Error)),

			self.rx_symbol  .eq(lane.rx_symbol),
			self.rx_valid   .eq(lane.rx_valid),
			self.tx_symbol  .eq(lane.tx_symbol),

			self.retry_buffer_occupation  .eq(phy.dll.status.retry_buffer_occupation),
			self.receive_buffer_occupation.eq(phy.dll.status.receive_buffer_occupation),
			self.tx_seq_num .eq(phy.dll.status.tx_seq_num),
			self.rx_seq_num .eq(phy.dll.status.rx_seq_num),
		]
		for f, sig in self.credits_rx.items():
			m.d.comb += sig.eq(getattr(phy.dll.credits_rx, f))

		tlp_rx = phy.dll_tlp_rx.tlp_source
		tlp_tx = phy.dll_tlp_tx.tlp_sink
		m.d.comb += [
			self.tlp_rx_data .eq(Cat(tlp_rx.symbol)),
			self.tlp_rx_valid.eq(Cat(tlp_rx.valid)),
		]
		if not self.transaction_layer:
			m.d.comb += [
				tlp_rx.ready.eq(self.tlp_rx_ready),
				Cat(tlp_tx.symbol).eq(self.tlp_tx_data),
				Cat(tlp_tx.valid).eq(self.tlp_tx_valid),
				self.tlp_tx_ready.eq(tlp_tx.ready),
			]

		return m
