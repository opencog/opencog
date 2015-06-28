#
# netcat.py - Quick-n-dirty network interface
# Copyright (C) 2015  Linas Vepstas
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License v3 as
# published by the Free Software Foundation and including the exceptions
# at http://opencog.org/wiki/Licenses
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program; if not, write to:
# Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.


import socket

# This implements netcat in python.
#
# If you don't now what netcat is, then you should google it.
# Its important and not complicated.
#
def netcat(hostname, port, content) :
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

	# If the cogserver is down, the connection will fail.
	try:
		s.connect((hostname, port))
	except socket.error as msg:
		print "Connect failed: ", msg
		s.close()
		return

	s.sendall(content)
	s.shutdown(socket.SHUT_WR)
	while True:
		data = s.recv(1024)
		if not data or data == "":
			break
		# print "Received:", repr(data)
	# print "Connection closed."
	s.close()
