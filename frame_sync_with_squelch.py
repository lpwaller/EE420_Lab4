#!/usr/bin/env python
from gnuradio import gr
from gnuradio import blocks
from gnuradio import digital
import numpy as np
import numpy
import matplotlib.pyplot as plt
import string_to_list
import time
import numpy


class frame_sync(gr.basic_block):
    def __init__(self, receiving_flag):
        gr.basic_block.__init__(self, name="frame_sync",
            in_sig=[numpy.uint8, numpy.float32],
            out_sig=[numpy.uint8])

        ##################################################
        # Put Variables Here
        ##################################################
        min_num_chars = 1
        max_num_chars = 32
        self.receiving_flag = receiving_flag
        self.barker13pre = [-1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1]
        self.barker13post = [-1, 1, -1, 1, -1, -1, 1, 1, -1, -1, -1, -1, -1]
        self.min_msg_len = 8*min_num_chars + 13 + 13 # min pkt length including pre and post (bits)
        self.max_msg_len = 8*max_num_chars + 13# max pkt payload plus post barker length (bits)

    def general_work(self, input_items, output_items):
        in0 = input_items[0]
        in1 = input_items[1]
        out = output_items[0]
        this = 0

        ninput_items = len(in0)
        if max(in1) < 0.5:
            self.consume(0, ninput_items)
            self.consume(1, len(in1))
            return 0;

        threshold = 11
        self.consume(1, int(len(in1)/2))

        if(ninput_items > self.min_msg_len): #if buffer is long enough
            in0_short = in0[0:ninput_items- self.min_msg_len]
            new_array = in0_short.astype(numpy.int32) * 2 - 1
            barkerprecheck = np.correlate(self.barker13pre, new_array)
            barkerprecheck = barkerprecheck[::-1]

            if (max(barkerprecheck) > threshold): #found a preamble
                start = np.argmax(barkerprecheck)+13 #move to the end of the barker code (start of packet)

                # try to find the end of the packet
                condensed_array = np.asarray(in0[start:]).astype(float) *2 -1
                potential_failure = False # track if there is a lone preamble and consume it.
                if len(condensed_array) > self.max_msg_len:
                    potential_failure = True
                    condensed_array = condensed_array[:self.max_msg_len]

                barkerpostcheck = np.correlate(self.barker13post, condensed_array)
                barkerpostcheck = barkerpostcheck[::-1]

                if (max(barkerpostcheck) > threshold): #found the end of the packet!
                    pkt_len = np.argmax(barkerpostcheck)

                    #check if it is a valid length (multiple of 8)
                    if np.mod(pkt_len,8) == 0: #Yay its a legit packet. pass it on.
                        for ndx in range(pkt_len):
                            out[ndx] = in0[start+ndx]
                        time.sleep(0.01)
                        self.receiving_flag[0] = 1;
                        self.consume(0,int(start) + int(pkt_len)+12) #consume to the end of the post barker
                        return int(pkt_len) #push a pkt

                    else: # Grr. Bad packet, not a valid set of chars (bad length)
                        self.consume(0,int(start) + int(pkt_len)+12) #consume the bad pkt to the end of the post barker
                        return 0 # dont push anything

                else: # did not find the end of the packet
                    if potential_failure: # found pre barker without post in searchable range
                        self.consume(0,int(start)-1) #consume the pre barker and look for a new one
                        return 0 # dont push anything
                    else:
                        self.consume(0,0) #not enough data, check for more
                        return 0

            else:
                self.consume(0, ninput_items-self.min_msg_len) # no prebarker in valid range. consume all
                #self.consume(0, (ninput_items-self.min_msg_len)*4)
                return 0
        else:
            self.consume(0,0) #not enough data, check for more
            return 0



