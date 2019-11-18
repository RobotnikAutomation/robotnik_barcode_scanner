#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys

import rospy

import time, threading

from rcomponent import RComponent, DEFAULT_FREQ, MAX_FREQ # esto podria estar como limites

import robotnik_msgs.msg

import serial

import std_msgs.msg

class BarcodeScannerComponent(RComponent):
    def __init__(self, args):
        RComponent.__init__(self, args)

        self.port = args['port']
        self.topic = args['topic']
        self.read_errors = 0
        self.serial_device = None
        self.barcode_publisher_ = None

    def setup(self):
        if self.initialized:
            rospy.logwarn("%s::setup: already initialized" % self.node_name)
            return 0

        RComponent.setup(self)

        rospy.loginfo("%s::setup" % self.node_name)

        self.serial_device = serial.Serial(
            port=self.port,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=1,
            bytesize=8,
            xonxoff=False,
            dsrdtr=False,
            rtscts=False
        )


    def shutdown(self):
        if self.running:
            rospy.logwarn("%st::shutdown: cannot shutdown because the component is still running" % self.node_name)
            return -1
        elif not self.initialized:
            rospy.logwarn("%s::shutdown: cannot shutdown because the component was not setup" % self.node_name)
            return -1

        RComponent.shutdown(self)

        rospy.loginfo("barcode::shutdown")

        if self.serial_device != None and not self.serial_device.closed:
            self.serial_device.close()

        return 0

    def rosSetup(self):
        if self.ros_initialized:
            rospy.logwarn("%s::rosSetup: already initialized" % self.node_name)
            return 0
        RComponent.rosSetup(self)

        self.barcode_publisher_ = rospy.Publisher(self.topic, std_msgs.msg.Header, queue_size=1)

    def rosShutdown(self):
        if self.running:
            rospy.logwarn("%s::rosShutdown: cannot shutdown because the component is still running" % self.node_name)
            return -1
        elif not self.ros_initialized:
            rospy.logwarn("%s::rosShutdown: cannot shutdown because the component was not setup" % self.node_name)
            return -1

        RComponent.rosShutdown(self)

        #self.set_digital_outputs_service_.shutdown()
        #self.get_sw_version_service_.shutdown()
        self.barcode_publisher_.unregister()

    def readyState(self):
        # currently we only have codes of 7 characters (plus 1 of end of line)
        size_of_code = 8
        code = self.readFromSerialDevice(size_of_code) # 8 bytes

        if size_of_code == len(code):
            msg = std_msgs.msg.Header()
            msg.frame_id = code[0:-1]
            self.barcode_publisher_.publish(msg)
        else:
            self.read_errors += 1

        max_read_errors = 20
        if self.read_errors >= max_read_errors:
            self.switchToState(State.FAILURE_STATE)

    def rosPublish(self):
        RComponent.rosPublish(self)

    def writeToSerialDevice(self, data):
        bytes_written = self.serial_device.write(data)
        return bytes_written

    def readFromSerialDevice(self, size):
        try:
            data_read = self.serial_device.read(size)
        except serial.SerialException:
            rospy.logwarn("Serial exception: Shutting node down")
            self.rosShutdown()

        return data_read

def main():
    rospy.init_node("barcode")

    _name = rospy.get_name().replace('/','')

    arg_defaults = {
      'topic_state': 'state',
      'topic': 'barcode_scanner',
      'desired_freq': DEFAULT_FREQ,
      'port' : '/dev/ttyACM0',
    }

    args = {}

    for name in arg_defaults:
        try:
            if rospy.search_param(name):
                args[name] = rospy.get_param('~%s'%(name)) # Adding the name of the node, because the para has the namespace of the node
            else:
                args[name] = arg_defaults[name]
            #print name
        except rospy.ROSException, e:
            rospy.logerr('%s: %s'%(e, _name))


    barcode_node = BarcodeScannerComponent(args)

    barcode_node.start()

if __name__ == "__main__":
    main()
