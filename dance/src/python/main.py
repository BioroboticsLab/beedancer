import BeeBoogie
import BeeSerial
import time
import numpy as np
from enum import Enum
import threading
import serial as ser
from matplotlib import pyplot as plt


class State(Enum):
	unCalibrated = 0
	Calibrated = 1
	Ready = 2
	Dancing = 3

class BeeController():
	def __init__(self, serial_link, dance_generator):
		self.serial_link = serial_link
		self.dance_generator = dance_generator
		self.current_state = State.unCalibrated
		self.current_position = [0.,0.,0.,0.]
		self.extract_position = [0.01859, 0.00681, 4.972, 0.]
		self.initial_dance_position = [0.01859, 0.00681, 0.32187]
		#_beedancer->goto_pos(0.01859, 0.00681, 4.972, 0., true);


	def how(self):
		self.current_state = State(int(self.serial_link.send_and_ack('how')))
		print(self.current_state)

	def close(self):
		self.serial_link.close_port()

	def go_to(self, x, y, t, df):
		self.serial_link.send_and_ack(str(3)+":"+"{:.5f}".format(x)+":"+"{:.5f}".format(y)+":"+"{:.5f}".format(t)+":"+"{:.5f}".format(df))
		self.current_position = [x,y,t,df]

	def dance(self):
		changed_initial = self.rotation(self.initial_dance_position, self.current_position[3])
		controller.go_to(changed_initial[0], changed_initial[1], changed_initial[2], self.current_position[3])
		data = self.dance_generator.get_pos_tab(30)
		data[:,0:2] = data[:,0:2]/1000
		for i in range(data.shape[0]):
			print(i)
			data[i,:] = self.rotation(data[i,:], self.current_position[3])
			if data[i, 2] < 0:
				data[i, 2] = 2 * np.pi + data[i, 2]
			data[i, 2] = data[i, 2] % (2* np.pi)
			#print(str(4)+":"+"{:.5f}".format(data[i, 0])+":"+"{:.5f}".format(data[i, 1])+":"+"{:.5f}".format(data[i, 2])+":"+"20")
			self.serial_link.send_and_ack(str(4)+":"+"{:.5f}".format(data[i, 0])+":"+"{:.5f}".format(data[i, 1])+":"+"{:.5f}".format(data[i, 2])+":"+"30")
		self.serial_link.send_and_ack(str(4)+":"+"1"+":"+"1"+":"+"1")
		plt.plot(data[:,0], data[:,1])
		plt.show()
		print("end of transmission")
		#print(str(3)+":"+"{:.5f}".format(data[0])+":"+"{:.5f}".format(data[1])+":"+"{:.5f}".format(data[2])+":"+"1000")
		#return self.serial_link.send_and_ack(str(4)+":"+"{:.5f}".format(data[0])+":"+"{:.5f}".format(data[1])+":"+"{:.5f}".format(data[2])+":"+"20")

	def calibrate(self):
		self.serial_link.send_and_ack("0")

	def extract(self):
		rotated_extraction_pos = self.rotation(self.extract_position, self.current_position[3])
		self.go_to(rotated_extraction_pos[0], rotated_extraction_pos[1], rotated_extraction_pos[2], self.current_position[3])
		self.serial_link.send_and_ack("1")

	def retract(self):
		rotated_extraction_pos = self.rotation(self.extract_position, self.current_position[3])
		self.go_to(rotated_extraction_pos[0], rotated_extraction_pos[1], rotated_extraction_pos[2], self.current_position[3])
		self.serial_link.send_and_ack("2")

	def rotate_dance_floor(self, angle, speed):
		time_step = 0.2
		principal_angle = self.get_principal_angle(angle)
		shortest_arc = self.get_shortest_arc(self.current_position[3], principal_angle)
		total_time = abs(shortest_arc) / speed
		nb_step = int(total_time / time_step)
		angular_step = shortest_arc / nb_step

		for i in range(nb_step):
			new_angle = self.current_position[3] + angular_step
			print("new angle :", new_angle)
			changed_position = self.rotation(self.current_position[:3], angular_step)
			print("changed_position", changed_position)
			self.go_to(changed_position[0], changed_position[1], changed_position[2], new_angle)
		

		changed_position = self.rotation(self.current_position[:3], angular_step)
		self.go_to(changed_position[0], changed_position[1], changed_position[2], angle)



	def rotation(self, vector, angle):
		xr = vector[0] * np.cos(angle) - vector[1] * np.sin(angle)
		yr = vector[0] * np.sin(angle) + vector[1] * np.cos(angle)
		tr = (vector[2] + angle) % (2 * np.pi)

		return xr, yr, tr 

	def get_principal_angle(self, angle):
		return (angle - np.pi) % (2 * np.pi) - np.pi

	def get_shortest_arc(self, current, target):
		directArc = 0.
		indirectArc = 0.
		if target < current:
			directArc = 2 * np.pi - (current - target)
			indirectArc = -(2 * np.pi - directArc)
		else:
			directArc = (target - current)
			indirectArc = -(2 * np.pi - directArc)

		if(abs(directArc)<abs(indirectArc)):
			return directArc
		else:
			return indirectArc 


def read_from_port(ser):
	while True:
		time.sleep(0.001)
		reading = ser.readline().decode('utf-8')
		if reading != "":
			print(reading)


if __name__=='__main__':
	password = 'beeboogie'
	serial = BeeSerial.SerialBeeBoogie(500000, password)

	#thread = threading.Thread(target=read_from_port, args=(serial.serial,))
	#thread.start()

	bee = BeeBoogie.BeeBoogie()
	bee.init_trajectory(15)
	controller = BeeController(serial, bee)
	#controller.how()

	#controller.go_to(0.01859, 0.00681, 0.32187)
	#time.sleep(4.)
	#print("wait")
	while(True):
		print("0 : Calibrate the robot || 1 : Extract the robot || 2 : Retract the robot || 3 : Orientate the DanceFloor || 4 : Make it Dance!")
		print("What do you want to do?")
		what = input()
		if what == "0":
			controller.calibrate()
		elif what == "1":
			controller.extract()
		elif what == "2":
			controller.retract()
		elif what == "3":
			print("Speed wanted (rad.s-1)?")
			speed = input()
			print("Position wanted (rad)")
			pos = input()
			controller.rotate_dance_floor(float(pos), float(speed))
		elif what == "4":
			controller.dance()



		#controller.dance()




