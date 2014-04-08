#!/usr/bin/env python
# encoding: utf-8
"""
YARPina.py

Copyright: 2013-2014 Juxi Leitner
Author: Juxi Leitner <juxi.leitner@gmail.com>
CopyPolicy: Released under the terms of the GNU GPL v2.0.
"""

import sys
import os
import socket
import string

class YARPina:
	"""docstring for YARPina"""	
	def __init__(self, ip = None, port = 10000):
		self.nameserver_ip = ip
		self.nameserver_port = port
		self.socket = None
		self.is_connected = False

	def init(self):
		"""

		Connects to the given nameserver (ip:port)

		Output: True/False whether succesfully connected or not
		"""

		if self.is_connected: 
			return True

		print "Trying to connect ..."
		try:
			# create socket
			self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.socket.connect((self.nameserver_ip, self.nameserver_port))

			# first command sent needs to be CONNECT
			print self.send("CONNECT YARPina")
			# if we are here, we are connected :)
			self.is_connected = True

			return True

		except Exception, e:
			# something's wrong
			print e
			self.is_connected = False
			self.socket = None
			return False

	def stop(self):
		self.is_connected = False		
		self.socket.close()
		self.socket = None

	def send(self, str):
		try:
			if str[-1] != '\n': str = str + '\n'
			if str[0:8] != "CONNECT ":
				# regular query has to start with d
				self.socket.send("d\n")
				
			self.socket.send(str)
			# do something more?
			return self.socket.recv(999)
		except Exception, e:
			raise e
			return False


	def query(self, portname):
		""" queries the nameserver for the ip, port of a named port"""
		b = Bottle(self.send("query " + portname))
		return (b.get_element_at(4), b.get_element_at(6))


	def port(self, target):
		return YARPPort(self.query(target))



class YARPPort:
	"""docstring for YARPPort"""
	def __init__(self, remote_addr):
		self.socket = None
		# todo check for tupel?
		self.remote_addr = remote_addr

	def send(self, b_in):
		try:
			# create socket
			self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.socket.connect((self.nameserver_ip, self.nameserver_port))
			return Bottle("PLACEHOLDER")
		except Exception, e:
			raise e

		

class Bottle(object):
	"""docstring for Bottle"""
	def __init__(self, str):
		self.content = str

	def get_element_at(self, at):
		tokens = self.content.split(" ")
		try:
			return tokens[at];
		except Exception, e:
			return ""

	# todo add more stuff in the future


if __name__ == '__main__':
	ip, port = "127.0.0.1", 10000
	print "YARPina: Testing to connect to YARP Nameserver @", ip, ":", port
	yarp = YARPina(ip, port)
	# check threading?	yarp.start()
	yarp.init()
	yarp.stop()
		