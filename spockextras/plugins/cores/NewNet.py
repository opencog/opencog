"""
Add a timestamp in data when reading packet

Original comment in net.py:
Provides an asynchronous, crypto and compression aware socket for connecting to
servers and processing incoming packet data.
Coordinates with the Timers plugin to honor clock-time timers
"""

import sys
import socket
import select
import time

from spock import utils
from spock.utils import pl_announce
from spock.mcp import mcpacket, mcdata
from Crypto.Cipher import AES
import rospy

import logging
logger = logging.getLogger('spock')

class AESCipher:
	def __init__(self, SharedSecret):
		#Name courtesy of dx
		self.encryptifier = AES.new(SharedSecret, AES.MODE_CFB, IV=SharedSecret)
		self.decryptifier = AES.new(SharedSecret, AES.MODE_CFB, IV=SharedSecret)

	def encrypt(self, data):
		return self.encryptifier.encrypt(data)

	def decrypt(self, data):
		return self.decryptifier.decrypt(data)

class SelectSocket:
	def __init__(self, timer):
		self.sending = False
		self.timer = timer
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.sock.setblocking(False)
		self.close = self.sock.close
		self.shutdown = self.sock.shutdown
		self.recv = self.sock.recv
		self.send = self.sock.send

	def poll(self):
		flags = []
		if self.sending:
			self.sending = False
			slist = [(self.sock,), (self.sock,), (self.sock,)]
		else:
			slist = [(self.sock,), (), (self.sock,)]
		timeout = self.timer.get_timeout()
		if timeout>=0:
			slist.append(timeout)
		try:
			rlist, wlist, xlist = select.select(*slist)
		except select.error as e:
			logger.error("Socket Error: %s", str(e))
			rlist = []
			wlist = []
			xlist = []
		if rlist:         flags.append('SOCKET_RECV')
		if wlist:         flags.append('SOCKET_SEND')
		if xlist:         flags.append('SOCKET_ERR')
		return flags

	def reset(self):
		self.sock.close()
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.sock.setblocking(False)

class NetCore:
	def __init__(self, sock, event, timer):
		self.sock = sock
		self.event = event
                self.timer = timer
		self.host = None
		self.port = None
		self.connected = False
		self.encrypted = False
		self.proto_state = mcdata.HANDSHAKE_STATE
		self.comp_state = mcdata.PROTO_COMP_OFF
		self.comp_threshold = -1
		self.sbuff = b''
		self.rbuff = utils.BoundBuffer()

	def connect(self, host = 'localhost', port = 25565):
		self.host = host
		self.port = port
		try:
			logger.info("Attempting to connect to host: %s port: %s", host, port)
			#Set the connect to be a blocking operation
			self.sock.sock.setblocking(True)
			self.sock.sock.connect((self.host, self.port))
			self.sock.sock.setblocking(False)
			self.connected = True
			self.event.emit('connect', (self.host, self.port))
			logger.info("Connected to host: %s port: %s", host, port)
		except socket.error as error:
			logger.error("Error on Connect: %s", str(error))

	def set_proto_state(self, state):
		self.proto_state = state
		self.event.emit(mcdata.state_lookup[state] + '_STATE')

	def set_comp_state(self, threshold):
		self.comp_threshold = threshold
		if threshold >=0:
			self.comp_state = mcdata.PROTO_COMP_ON

	def push(self, packet):
#                print '[net.py]push packet.ident %s'%(str(packet.ident))
#                print '[net.py]push packet.str_ident %s'%(packet.str_ident)
		data = packet.encode(self.comp_state, self.comp_threshold)
		self.sbuff += (self.cipher.encrypt(data) if self.encrypted else data)
		self.event.emit(packet.ident, packet)
		self.event.emit(packet.str_ident, packet)
		self.sock.sending = True

	def push_packet(self, ident, data):
		self.push(mcpacket.Packet(ident, data))

	def read_packet(self, data = b''):
		self.rbuff.append(self.cipher.decrypt(data) if self.encrypted else data)
		while True:
			self.rbuff.save()
			try:
				packet = mcpacket.Packet(ident = (
					self.proto_state,
					mcdata.SERVER_TO_CLIENT,
				)).decode(self.rbuff, self.comp_state)
			except utils.BufferUnderflowException:
				self.rbuff.revert()
				break
			except mcpacket.PacketDecodeFailure as err:
				logger.warning('Packet decode failed')
				logger.warning(
					'Failed packet ident is probably: %s', err.packet.str_ident
				)
				self.event.emit('PACKET_ERR', err)
				break
                        #print '[net.py]read packet.ident %s'%(str(packet.ident))
                        #print '[net.py]read packet.str_ident %s'%(packet.str_ident)
                        rostime=rospy.Time.now()
                        packet.data['ROStimestamp']=rostime.secs*10e9+rostime.nsecs
                        packet.data['MCtimestamp']=self.timer.world.time_of_day
			self.event.emit(packet.ident, packet)
			self.event.emit(packet.str_ident, packet)

	def enable_crypto(self, secret_key):
		self.cipher = AESCipher(secret_key)
		self.encrypted = True

	def disable_crypto(self):
		self.cipher = None
		self.encrypted = False

	def reset(self):
		self.connected = False
		self.sock.reset()
		self.__init__(self.sock, self.event)

	disconnect = reset

default_settings = {
	'username': 'Bot',
	'password': None,
	'bufsize': 4096,
	'sock_quit': True,
	'sess_quit': True,
}

@pl_announce('Net')
class NetPlugin:
	def __init__(self, ploader, settings):
		settings = utils.get_settings(settings, default_settings)
		self.bufsize = settings['bufsize']
		self.sock_quit = settings['sock_quit']
		self.event = ploader.requires('Event')
		self.timer = ploader.requires('Timers')
		self.sock = SelectSocket(self.timer)
		self.net = NetCore(self.sock, self.event,self.timer)
		self.sock_dead = False
		ploader.provides('Net', self.net)

		ploader.reg_event_handler('event_tick', self.tick)
		ploader.reg_event_handler('SOCKET_RECV', self.handleRECV)
		ploader.reg_event_handler('SOCKET_SEND', self.handleSEND)
		ploader.reg_event_handler('SOCKET_ERR', self.handleERR)
		ploader.reg_event_handler('SOCKET_HUP', self.handleHUP)
		ploader.reg_event_handler('PLAY<Disconnect', self.handle_disconnect)
		ploader.reg_event_handler('HANDSHAKE>Handshake', self.handle_handshake)
		ploader.reg_event_handler('LOGIN<Login Success', self.handle_login_success)
		ploader.reg_event_handler('LOGIN<Set Compression', self.handle_comp)
		ploader.reg_event_handler('PLAY<Set Compression', self.handle_comp)
		ploader.reg_event_handler('kill', self.handle_kill)

	def tick(self, name, data):
		if self.net.connected:
			for flag in self.sock.poll():
				self.event.emit(flag)
		else:
			timeout = self.timer.get_timeout()
			if timeout == -1:
				time.sleep(1)
			else:
				time.sleep(timeout)


	#SOCKET_RECV - Socket is ready to recieve data
	def handleRECV(self, name, data):
		if self.net.connected:
			try:
				data = self.sock.recv(self.bufsize)
				#print('read:', len(data))
				if not data: #Just because we have to support socket.select
					self.event.emit('SOCKET_HUP')
					return
				self.net.read_packet(data)
			except socket.error as error:
				self.event.emit('SOCKET_ERR', error)


	#SOCKET_SEND - Socket is ready to send data and Send buffer contains data to send
	def handleSEND(self, name, data):
		if self.net.connected:
			try:
				sent = self.sock.send(self.net.sbuff)
				self.net.sbuff = self.net.sbuff[sent:]
				if self.net.sbuff:
					self.sending = True
			except socket.error as error:
				logger.error(str(error))
				self.event.emit('SOCKET_ERR', error)

	#SOCKET_ERR - Socket Error has occured
	def handleERR(self, name, data):
		self.net.reset()
		logger.error("Socket Error: %s", data)
		self.event.emit('disconnect', data)
		if self.sock_quit and not self.event.kill_event:
			self.sock_dead = True
			self.event.kill()

	#SOCKET_HUP - Socket has hung up
	def handleHUP(self, name, data):
		self.net.reset()
		logger.error("Socket has hung up")
		self.event.emit('disconnect', "Socket Hung Up")
		if self.sock_quit and not self.event.kill_event:
			self.sock_dead = True
			self.event.kill()

	#Handshake - Change to whatever the next state is going to be
	def handle_handshake(self, name, packet):
		self.net.set_proto_state(packet.data['next_state'])

	#Login Success - Change to Play state
	def handle_login_success(self, name, packet):
		self.net.set_proto_state(mcdata.PLAY_STATE)

	#Handle Set Compression packets
	def handle_comp(self, name, packet):
		self.net.set_comp_state(packet.data['threshold'])

	def handle_disconnect(self, name, packet):
		logger.info("Disconnected: %s", packet.data['reason'])
		self.event.emit('disconnect', packet.data['reason'])

	#Kill event - Try to shutdown the socket politely
	def handle_kill(self, name, data):
		logger.info("Kill event recieved, shutting down socket")
		if not self.sock_dead:
			self.sock.shutdown(socket.SHUT_WR)
		self.sock.close()
