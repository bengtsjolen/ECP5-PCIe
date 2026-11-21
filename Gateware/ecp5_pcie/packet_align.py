from amaranth import *
from amaranth.build import *

from .serdes import Ctrl
from .stream import StreamInterface


class PCIePacketAligner(Elaboratable):
	"""
	Shifts the received symbol stream so that every STP and SDP lands in lane 0.

	The comma aligner only aligns ordered sets. Packets can start in any lane, since the link
	partner sends any number of idle symbols between them and clock compensation adds or removes
	SKP symbols. TLPs and DLLPs are multiples of 4 symbols long, so the offset of a packet start
	holds until the next one. Output lags input by one word.
	"""
	def __init__(self, ratio = 4):
		self.sink = StreamInterface(9, ratio, name="Align_Sink")
		self.source = StreamInterface(9, ratio, name="Align_Source")
		self.offset = Signal(range(ratio))

	def elaborate(self, platform: Platform) -> Module:
		m = Module()

		ratio = len(self.sink.symbol)

		prev = [Signal(9, name=f"prev_{i}") for i in range(ratio)]
		prev_valid = [Signal(name=f"prev_valid_{i}") for i in range(ratio)]
		m.d.rx += [prev[i].eq(self.sink.symbol[i]) for i in range(ratio)]
		m.d.rx += [prev_valid[i].eq(self.sink.valid[i]) for i in range(ratio)]
		m.d.comb += self.sink.ready.eq(self.source.ready)

		# A word holds at most one packet start, since packets are at least 8 symbols long.
		start = Signal(range(ratio))
		found = Signal()
		for i in reversed(range(ratio)):
			with m.If((prev[i] == Ctrl.STP) | (prev[i] == Ctrl.SDP)):
				m.d.comb += [start.eq(i), found.eq(1)]

		offset = Signal(range(ratio))
		m.d.comb += offset.eq(Mux(found, start, self.offset))
		m.d.rx += self.offset.eq(offset)

		window = prev + self.sink.symbol
		window_valid = prev_valid + self.sink.valid
		with m.Switch(offset):
			for k in range(ratio):
				with m.Case(k):
					for i in range(ratio):
						m.d.comb += [
							self.source.symbol[i].eq(window[k + i]),
							self.source.valid[i].eq(window_valid[k + i]),
						]

		return m
