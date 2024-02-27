##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2014 Gump Yang <gump.yang@gmail.com>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd

class SamplerateError(Exception):
    pass

class Decoder(srd.Decoder):
    api_version = 3
    id = 'ir_gree'
    name = 'IR GREE'
    longname = 'IR GREE'
    desc = 'GREE YAPOF infrared remote control protocol'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = []
    tags = ['IR']
    channels = (
        {'id': 'ir', 'name': 'IR', 'desc': 'Data line'},
    )
    options = ()
    annotations = (
        ('bit', 'Bit'),
        ('leader-code', 'Leader Code'),
        ('connect-code', 'Connect Code'),
    )
    annotation_rows = (
        ('bits', 'Bits', (0, 1, 2)),
    )

    def __init__(self):
        self.reset()

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def reset(self):
        self.samplerate = None
        self.sample_start = 0

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value
        # 采样率 * 时间 = 采样数
        # 引导码 Low 9ms + High 4.5ms=13.5ms, 即0.0135s
        # 上下浮动10%
        self.leader_start = int(self.samplerate * 0.012)
        self.leader_end = int(self.samplerate * 0.015)

        # 连接码 Low 0.66ms + High 20ms
        self.connect_start = int(self.samplerate * 0.0205)
        self.connect_end = int(self.samplerate * 0.0208)

        # 0 Low 0.66ms + High 0.54ms = 1.2ms
        self.zero_start = int(self.samplerate * 0.00108)
        self.zero_end = int(self.samplerate * 0.00144)

        # 1 Low 0.66ms + High 1.66ms = 2.32ms
        self.one_start = int(self.samplerate * 0.00208)
        self.one_end = int(self.samplerate * 0.00255)

    def decode(self):
        while True:
            self.wait({0: 'f'})
            if (self.sample_start == 0):
                self.sample_start = self.samplenum
            else:
                b = self.samplenum - self.sample_start
                if (b > self.leader_start and b < self.leader_end):
                    self.put(self.sample_start, self.samplenum, self.out_ann, [1,['Leader code', 'Leader', 'L']])
                elif (b > self.connect_start and b < self.connect_end):
                    self.put(self.sample_start, self.samplenum, self.out_ann, [2,['Connect Code', 'Connect', 'C']])
                elif (b > self.zero_start and b < self.zero_end):
                    self.put(self.sample_start, self.samplenum, self.out_ann, [0,['%d' % 0]])
                elif (b > self.one_start and b < self.one_end):
                    self.put(self.sample_start, self.samplenum, self.out_ann, [0,['%d' % 1]])
                    
                self.sample_start = self.samplenum