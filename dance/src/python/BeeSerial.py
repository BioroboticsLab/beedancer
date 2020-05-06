import serial.tools.list_ports
import sys
import glob
import time


from time import sleep
import serial
'''
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1) # Establish the connection on a specific port
counter = 0# Below 32 everything in ASCII is gibberish
sleep(2)
while True:
	counter +=1
	ser.write(bytes(str(counter)+'\n', 'utf-8')) # Convert the decimal number to ASCII then send it to the Arduino
	print('loop', ser.readline().decode('utf-8')) # Read the newest output from the Arduino
	sleep(.1) # Delay for one tenth of a second
	if counter == 255:
		counter = 32
'''

class SerialBeeBoogie():
	def __init__(self, baudrate, password):
		self.baudrate = baudrate
		self.password = password
		self.timeout = 1
		self.port = ''
		self.findport()
		self.serial = None
		self.open_port(self.port)

		self.close_port()

	def open_port(self, port):
		self.serial = serial.Serial(port = port, baudrate = self.baudrate, timeout = self.timeout)
		time.sleep(2)
		print(self.serial.port, 'is now open')
		return self.serial

	def close_port(self):
		self.serial.close()
		print(self.serial.port, 'is now closed')

	def findport(self):
		# Finds the serial port names
		if sys.platform.startswith('win'):
			ports = ['COM%s' % (i+1) for i in range(256)]
		elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
			ports = glob.glob('/dev/ttyUSB*')
		elif sys.platform.startswith('darwin'):
			ports = glob.glob('/dev/tty.usbserial*')
		else:
			raise EnvironmentError('Error finding ports on your operating system')

		for p in ports:
			s = self.open_port(p)
			is_bee = self.isBee(s)
			s.close()

			if is_bee:
				self.port = p

		if self.port == '':
			raise OSError('Cannot find the bee dancer')
		else:
			print('Bee dancer founded in port :', self.port)


	def isBee(self, serial):
		"""
		When automatically detecting port, parse the serial return for the "OpenBCI" ID.
		"""
		#Wait for device to send data
		line = ''
		q=bytes('who'+'\n', 'utf-8')
		serial.write(q)
		c = serial.readline().decode('utf-8')
		line += c
		print("Password returned : ", line)
		if self.password in line:
			return True
		return False

	def send_and_ack():
		return 0
		

if __name__=='__main__':
	password = 'beeboogie'
	serial = SerialBeeBoogie(115200, password)