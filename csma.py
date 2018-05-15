#!/usr/bin/env python
from gnuradio import gr
from gnuradio import blocks
from gnuradio import digital
from gnuradio import analog
from gnuradio import uhd
import gnuradio.gr.gr_threading as _threading
from optparse import OptionParser
from gnuradio import eng_notation
from gnuradio.eng_option import eng_option
from frame_sync_with_squelch import frame_sync
import time
import sys
from numpy.random import exponential

class my_top_block(gr.top_block):
    def __init__(self, rx_callback, freq, tx_status, rx_status):
        gr.top_block.__init__(self)

        ##################################################
        # Variables
        ##################################################
        self.freq = freq
        self.samp_rate = 1e6
        self.rxgain = 15
        self.txgain = 15

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_sink = uhd.usrp_sink(
            device_addr="",
            stream_args=uhd.stream_args(
                cpu_format="fc32",
                channels=range(1),
            ),
        )
        self.uhd_usrp_sink.set_samp_rate(self.samp_rate)
        self.uhd_usrp_sink.set_center_freq(self.freq, 0)
        self.uhd_usrp_sink.set_gain(self.rxgain, 0)

        self.uhd_usrp_source = uhd.usrp_source(
            device_addr="",
            stream_args=uhd.stream_args(
                cpu_format="fc32",
                channels=range(1),
            ),
        )
        self.uhd_usrp_source.set_samp_rate(self.samp_rate)
        self.uhd_usrp_source.set_center_freq(self.freq, 0)
        self.uhd_usrp_source.set_gain(self.txgain, 0)

        self.txpath = transmit_path()
        self.rxpath = receive_path(rx_callback, tx_status, rx_status)


        ##################################################
        # Connections
        ##################################################

        self.connect(self.txpath, self.uhd_usrp_sink)
        self.connect(self.uhd_usrp_source, self.rxpath)

    def send_pkt(self, payload='', eof=False):
        return self.txpath.send_pkt(payload, eof)

class transmit_path(gr.hier_block2):
    def __init__(self):
        gr.hier_block2.__init__(self, "transmit_path",
            gr.io_signature(0,0,0),
            gr.io_signature(1, 1, gr.sizeof_gr_complex))

        ##################################################
        # Variables
        ##################################################
        self.msgq_limit = msgq_limit = 2

        ##################################################
        # Blocks
        ##################################################

        self.pkt_input = blocks.message_source(gr.sizeof_char, msgq_limit)

        # self.input_unpacked_to_packed = blocks.unpacked_to_packed_bb(1, gr.GR_MSB_FIRST)

        self.mod = digital.gfsk_mod(
        	samples_per_symbol=4,
        	sensitivity=1.0,
        	bt=0.35,
        	verbose=False,
        	log=False,
        )

        ##################################################
        # Connections
        ##################################################
        self.connect(self.pkt_input, self.mod, self)

    def send_pkt(self, payload='', eof=False):
        if eof:
            msg = gr.message(1)  # tell self._pkt_input we're not sending any more packets
        else:
            msg = gr.message_from_string(payload)
            #print("sent a message")
            sys.stdout.flush()

        self.pkt_input.msgq().insert_tail(msg)

class receive_path(gr.hier_block2):
    def __init__(self, rx_callback, tx_status, rx_status):
        gr.hier_block2.__init__(self, "receive_path",
            gr.io_signature(1, 1, gr.sizeof_gr_complex),
            gr.io_signature(0, 0, 0))

        ##################################################
        # Variables
        ##################################################
        self.callback = rx_callback

        ##################################################
        # Blocks
        ##################################################
        self.demod = digital.gfsk_demod(
        	samples_per_symbol=4,
        	sensitivity=1.0,
        	gain_mu=0.175,
        	mu=0.5,
        	omega_relative_limit=0.005,
        	freq_error=0.0,
        	verbose=False,
        	log=False,
        )
        self.frame_sync = frame_sync(tx_status, rx_status) # Your custom block!!!

        self.output_unpacked_to_packed = blocks.unpacked_to_packed_bb(1, gr.GR_MSB_FIRST)

        self.rcvd_pktq = gr.msg_queue()
        self.message_sink = blocks.message_sink(gr.sizeof_char, self.rcvd_pktq, True)

        self.queue_watcher_thread = _queue_watcher_thread(self.rcvd_pktq, self.callback)

        self.blocks_max = blocks.max_ff(1,1)

        #Don't Search if nothing is transmitting
        self.blocks_threshold_ff_0 = blocks.threshold_ff(0.00002, 0.00002, 0)
        self.blocks_complex_to_mag_squared_0 = blocks.complex_to_mag_squared(1)
        self.analog_simple_squelch_cc_0 = analog.simple_squelch_cc(-80, 0.55)
        ##################################################
        # Connections
        ##################################################
        #main path
        self.connect(self, self.demod, (self.frame_sync, 0))
        self.connect(self.frame_sync, self.output_unpacked_to_packed, self.message_sink)

        #squelch path
        self.connect(self, (self.analog_simple_squelch_cc_0, 0), (self.blocks_complex_to_mag_squared_0, 0))
        self.connect((self.blocks_complex_to_mag_squared_0, 0), (self.blocks_max, 0), (self.blocks_threshold_ff_0, 0))
        self.connect((self.blocks_threshold_ff_0, 0), (self.frame_sync, 1))

class _queue_watcher_thread(_threading.Thread):
    def __init__(self, rcvd_pktq, callback):
        _threading.Thread.__init__(self)
        self.setDaemon(1)
        self.rcvd_pktq = rcvd_pktq
        self.callback = callback
        self.keep_running = True
        self.start()

    def run(self):
        while self.keep_running:
            msg = self.rcvd_pktq.delete_head()
            payload = msg.to_string()
            if self.callback:
                self.callback(payload)

class tdd_mac(object):
    def __init__(self):
        self.tb = None
        # barkerpre = [0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0]
        # need it to be two chars because it is length 13 and two chars is 16. prepend three 0s and
        # split into two bytes: [00000000] = 0 and [11001010] =  2+8+64+128 = 202
        self.barker13pre = chr(0) + chr(202)
        self.ack = chr(0)

        # barkerpost =  [0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0]
        # similar process, but 3 extra 0s go on the end now. [01010011] = 1+2+16+64 = 83 , [00000000]
        self.barker13post = chr(83) + chr(0)

        self.empty_frame_size = 1472

        empty_frame = ""
        for ndx in range(self.empty_frame_size):
            empty_frame = empty_frame + chr(0)
        self.empty_frame = empty_frame

        self.ack_msg = self.empty_frame+self.barker13pre+self.ack+self.barker13post+self.empty_frame


    def set_top_block(self, tb):
        self.tb = tb

    def rx_callback(self, payload):
        # Stuff goes here like:
        print("Breaking News: " + payload)

    def main_loop(self, tx_status, rx_status):
        print("\n\nType something to send: ")
        user_msg = raw_input()
        fill_frame = ""
        for ndx in range((len(user_msg)+4),self.empty_frame_size):
            fill_frame = fill_frame + chr(0)
        full_msg = self.empty_frame+self.empty_frame+self.barker13pre+user_msg+self.barker13post+fill_frame


        while 1:
            ##################################################
            # Your Mac logic goes here                       #
            ##################################################
            # Default send frame size is 1472 (maybe. I'm not sure. ref: http://lists.ettus.com/pipermail/usrp-users_lists.ettus.com/2011-April/001040.html
            if rx_status[1] == 1:
                self.tb.send_pkt(self.ack_msg)

            #ack received. Reset
            if tx_status[5] == 1:
                print("received ack")
                tx_status[0] = 2000
                tx_status[1] = 0
                tx_status[2] = 2000
                tx_status[3] = 0
                tx_status[4] = 0
                tx_status[5] = 0

            # No packet ready to send
            if tx_status[0] > 0:
                tx_status[0] += -1

            # Packet ready, check channel
            elif tx_status[1] == 0:
                print("Initiating Spectral Scan")
                tx_status[1] = 1
                tx_status[2] = 2000

            #channel ready
            elif tx_status[2] == 0:
                #send pkt
                if tx_status[4] == 0:
                    print("Sending Packet")
                    self.tb.send_pkt(full_msg)
                    #start timeout for packet drop
                    tx_status[4] = 2000
                    print("Awaiting ack")
                else: # awaiting ack
                    tx_status[4] += -1
                    time.sleep(0.001)


            # The receiver thread automatically handles receiving


def main():
    rx_status = [0,0] # Channel Empty, Send Ack
    tx_status = [1000,0,1000,0,0,0] #0: Packet Timer, 1: Ready, 2: Channel Ready, 3: Backoff Timer, 4: Send Packet, 5: Recieved ACK

    parser = OptionParser(option_class=eng_option, conflict_handler="resolve")
    parser.add_option("-f", "--freq", type="eng_float", default=None,
                          help="set center frequency to FREQ", metavar="FREQ")
    parser.add_option("-m", type="eng_float", default=1,
                        help = "set mode, tx or rx", metavar="MODE")

    (options, args) = parser.parse_args()

    #freq = options.freq
    freq = 5.45e9
    print "Freq = " + str(freq)

    if freq is None:
        raise ValueError("Center frequency not specified...")

    r = gr.enable_realtime_scheduling()
    if r != gr.RT_OK:
        print "Warning: failed to enable realtime scheduling"

    mac = tdd_mac()

    tb = my_top_block(mac.rx_callback, freq, tx_status, rx_status)

    mac.set_top_block(tb)

    tb.start() # Start executing the flow graph (runs in separate threads)

    mac.main_loop(tx_status, rx_status) # don't expect this to return

    tb.stop() # but if it does, tell flow graph to stop.
    tb.wait() # wait for it to finish

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
