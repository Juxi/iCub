#!/usr/bin/env python
# encoding: utf-8
"""
untitled.py

Copyright: 2013-2014 Juxi Leitner
Author: Juxi Leitner <juxi.leitner@gmail.com>
CopyPolicy: Released under the terms of the GNU GPL v2.0.

"""

import sys
import os
from YARPina import *

def main():
	# connect to YARP server
	ip, port = "127.0.0.1", 10000
	yarp = YARPina(ip, port)
	yarp.init()

	# open test port (for querying) and connect it to to /root
	print "trying to open port connected to /root"
	port = yarp.port( target = "/root" )
	print port.remote_addr
	port.send(Bottle("test"))

	pass


if __name__ == '__main__':
	main()

