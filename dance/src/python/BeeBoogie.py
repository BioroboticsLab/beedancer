import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import copy
import json
from scipy.interpolate import interp1d
import numpy as np
import svgwrite

class BeeBoogie():
	'''
	This class is inspired by the work of Tim Landgraf, it produce a trajectory 
	callable to control a robot bee to reproduce waggle dance in bee hive 
	'''
	def __init__(self, config_json = 'config.json', return_path = 'return_origin.npy'):
		'''
		Constructor
		:config_json: path of the config file
		:return_path: path of the return origin trajectory
		'''

		with open(config_json, 'r') as myfile:
			data=myfile.read()

		self.config = json.loads(data)
		self.return_interpolation = np.load(return_path, allow_pickle=True).item()

		self.robot_trajectory = np.array([])
		self.bee_trajectory = np.array([])
		self.trajectory_ready = False
		self.current_waggle_trajectory = 0

		ramp = 0.1
		self.stretch_function = lambda x : x if 0 <= x <= 0.5 else ((1-x) if 0.5 < x <= 1 else 0)
		self.speed_ramp = lambda x : (x if 0 <= x <= ramp else ((1-x) if 1-ramp < x <= 1 else ((ramp) if ramp < x <= 1-ramp else 0)))*1/ramp
		self.waggle_function_list = self.waggle_generator(self.speed_ramp)

		x_max = -self.waggle_function_list[0]['inter_x_bee'](self.waggle_function_list[0]['duration'])
		theta_goal = 2*np.pi + self.waggle_function_list[0]['inter_theta_bee'](0)
		y_goal = 0.0
		goal = (x_max, y_goal, theta_goal)

		self.return_function = self.return_generator(self.speed_ramp, goal)
		self.current_position_step = 0
		self.total_step = 0
		self.total_duration = 0

		self.addr_return_1 = 0
		self.addr_return_2 = 0
		self.addr_waggle_1 = 0
		self.addr_waggle_2 = 0


	def save_waggle_list(self, file_name='waggle_1-14.npy'):
		'''
		function used to save the waggle trajectory which can be computer intensive
		'''
		np.save(file_name, self.waggle_function_list)

	def waggle_generator(self, speed_ramp):
		'''
		This function generate a list of dict. each dict contain the interpolation function of the trjectory with waggle
		nb_waggle_max : Max number of waggle, will compute from 1 to nb_max trajectory.
		speed_ramp : speed ramp of the trajectory.
		config_json : title of the json file containing the config
		return: list n=0 : one waggle ; n=14 : fifteen waggle 
		key : 'inter_x_bee', 'inter_y_bee', 'inter_theta_bee', 'inter_x_robot', 'inter_y_robot', 'inter_theta_robot', 'duration'
		'''
		def get_waggle_ramp_tab(nb_waggle, nb_steps, waggle_amp):
			'''
			The waggle amplitude is not regular it increase and decrease, this function compute 
			the amplitude of the waggle for each waggle configuration
			:nb_waggle: number of waggle to compute
			:nb_steps: number of step to compute
			:waggle_amp: max amplitude to reach of the waggle 
			'''
			waggle_ramp_length_normalize = waggle_ramp_length / duration
			waggle_length_normalize = nb_waggle / (nb_waggle_max + extra_waggle)
			waggle_ramp = lambda x : (x if 0 <= x <= waggle_ramp_length_normalize 
								  else ((waggle_length_normalize-x) if waggle_length_normalize-waggle_ramp_length_normalize < x <= waggle_length_normalize 
										else ((waggle_ramp_length_normalize) if waggle_ramp_length_normalize < x <= waggle_length_normalize - waggle_ramp_length_normalize
											  else 0)))*1/waggle_ramp_length_normalize
			time_vector = np.linspace(0,1,nb_steps)
			waggle_ramp_tab = np.zeros(nb_steps)
			for i in range(steps):
				waggle_ramp_tab[i]=waggle_ramp(time_vector[i]) #* waggle_amp
			
			return waggle_ramp_tab
		
		def rotation(angle, x, y, theta):
			'''
			easy
			'''
			theta = theta + angle
			c, s = np.cos(angle), np.sin(angle)
			R = np.array([[c,-s],[s,c]])
			coord = np.vstack([x,y])
			coord = np.dot(R, coord)
			
			return coord[0,:], coord[1,:], theta
			
		#Get all the parameter
		nb_waggle_max = self.config['nb_waggle_max']
		waggle_divergence = self.config['waggle_divergence_rad']
		waggle_speed = self.config['waggle_speed_mms-1']
		waggle_freq = self.config['waggle_freq_hz']
		waggle_amp = self.config['waggle_amp_a_rad']
		waggle_ramp_length = (self.config['waggle_ramp_length_ms'] / 1000)
		exc = self.config['exc_mm']
		
		extra_waggle = 2
		steps = self.config['interpolation_step']
		
		#Get the total duration of the waggle part with the waggle freq
		duration = (nb_waggle_max + extra_waggle) / waggle_freq # + 2 for the ramp
		
		time_vector = np.linspace(0,1,steps)
		
		#Get the speed ramp
		speed_ramp_tab = np.zeros(steps)
		for i in range(steps):
			speed_ramp_tab[i]=self.speed_ramp(time_vector[i]) * waggle_speed
		
		#Get the total length of the waggle part
		length = np.trapz(speed_ramp_tab, dx=(1/steps)*duration)
		
		#Init
		x_robot = np.zeros(steps)
		y_robot = np.zeros(steps)
		theta_robot = np.zeros(steps)
		x_bee = np.zeros(steps)
		y_bee = np.zeros(steps)
		theta_bee = np.zeros(steps)
		waggle_function_list = []
		time_vector_duration = duration * time_vector
		
		for i in range(1, nb_waggle_max+1):
			#Generate as many interpolation as their is waggle 
			#if waggle max is 5, we have to generate 1,2,3,4,5 waggle
			x_robot = np.zeros(steps)
			y_robot = np.zeros(steps)
			theta_robot = np.zeros(steps)
			x_bee = np.zeros(steps)
			y_bee = np.zeros(steps)
			theta_bee = np.zeros(steps)
			
			function_dict = dict()
			
			waggle_ramp_tab = get_waggle_ramp_tab(i + extra_waggle, steps, waggle_amp)
			for j in range(0, steps):
				#Generate angle and position for each time step
				x_robot[j] = np.trapz(speed_ramp_tab[:j], dx=(1/steps)*duration) + exc
				theta_bee[j] = theta_robot[j] = np.sin(2*np.pi*j*(1/(steps/(nb_waggle_max + extra_waggle)))) * waggle_ramp_tab[j] * waggle_amp
				x_bee[j] = x_robot[j] - exc * np.cos(theta_bee[j])
				y_bee[j] = y_robot[j] - exc * np.sin(theta_bee[j])
				
			x_bee, y_bee, theta_bee = rotation(-waggle_divergence/2, x_bee, y_bee, theta_bee)
			x_robot, y_robot, theta_robot = rotation(-waggle_divergence/2, x_robot, y_robot, theta_robot) 
			
			#Produce all the interpolation for being able to change timestep if needed
			function_dict['inter_x_bee'] = interp1d(time_vector_duration, x_bee, kind='linear')
			function_dict['inter_y_bee'] = interp1d(time_vector_duration, y_bee, kind='linear')
			function_dict['inter_theta_bee'] = interp1d(time_vector_duration, theta_bee, kind='linear')
			
			function_dict['inter_x_robot'] = interp1d(time_vector_duration, x_robot, kind='linear')
			function_dict['inter_y_robot'] = interp1d(time_vector_duration, y_robot, kind='linear')
			function_dict['inter_theta_robot'] = interp1d(time_vector_duration, theta_robot, kind='linear')
			function_dict['duration'] = duration
			
			waggle_function_list.append(function_dict)
			
		return waggle_function_list

	def return_generator(self, speed_ramp, goals):
		'''
		This function take a model trajectory (return_path) and stretch it to make a close trajectory at the end
		:Speed_ramp: speed ramp of the fake bee
		:goals: (x_goal, y_goal, theta_goal) goals of the stretch
		'''
		return_speed = self.config['return_speed_mms-1']
		waggle_divergence = self.config['waggle_divergence_rad']
		exc = self.config['exc_mm']
		steps = self.config['interpolation_step']
				
		def stretch_function_normalize(n_step, function):
			t = np.linspace(0, 1, n_step)
			farr = t

			for i in range(t.shape[0]):
				farr[i] = function(t[i])

			integral_f = sum(farr)

			farr = farr/integral_f

			return farr
			
		x_goal, y_goal, theta_goal = goals
			
		t = np.linspace(0, 1, steps)
		vector = np.transpose(np.array([self.return_interpolation['inter_x_bee'](t), 
										self.return_interpolation['inter_y_bee'](t),
										self.return_interpolation['inter_theta_bee'](t)]))
			
		delta_theta_init = waggle_divergence / 2 - vector[0, 2]
		vector[:, 2] = vector[:, 2] + delta_theta_init
			
		#Definition of all the deltas to compensate
		deltas = np.array([vector[-1, 0] - x_goal, vector[-1, 1] - y_goal, vector[-1, 2] - theta_goal])
			
		#normalization of the given stretch function (integrale[0->1] = 1)
		stretch_function_normalized = stretch_function_normalize(vector.shape[0], self.stretch_function)
			
		#Out vector
		vector_rescaled = np.array(vector)
			
		#Stretiching
		for i in range(vector.shape[0]):
			vector_rescaled[i, 0] = vector[i, 0] - sum(deltas[0] * stretch_function_normalized[:i])
			vector_rescaled[i, 1] = vector[i, 1] - sum(deltas[1] * stretch_function_normalized[:i])
			vector_rescaled[i, 2] = vector[i, 2] - sum(deltas[2] * stretch_function_normalized[:i])
			
		#Length of the trajectory for deducting the duration
		integral_length = np.zeros(vector_rescaled.shape[0]-1)
		for i in range(0, vector_rescaled.shape[0]-1):
			integral_length[i] = sum(np.linalg.norm(vector_rescaled[:i+1,:-1] - vector_rescaled[1:i+2,:-1], axis=1))
		integral_length[0] = 0.0001 
			
		total_path_length = sum(np.linalg.norm(vector_rescaled[:-1,:-1] - vector_rescaled[1:,:-1], axis=1))
		total_return_duration = 1
			
		#We stretch the time vector to have a uniform repartition of the point (ex 1ms = 1mm)
		# Function : tn = (sum(dn)_0^K * Ttot)/Dtot
		# stretched_time_vector = np.zeros(stretch_function_normalized.shape[0])
		stretched_time_vector = np.zeros(stretch_function_normalized.shape[0])
		for i in range(1, stretched_time_vector.shape[0]):
			stretched_time_vector[i] = integral_length[i-1] * total_return_duration / integral_length[-1]
		vector_rescaled[-1,:] = [x_goal, y_goal, theta_goal]
			
		speed_ramp_tab = np.zeros(steps)
		for i in range(steps):
			speed_ramp_tab[i] = self.speed_ramp(t[i]) * return_speed
					
		duration = integral_length[-1] / np.trapz(speed_ramp_tab, dx=1/steps)
			
		stretched_time_vector = stretched_time_vector * duration
			
		Xinterpolation = interp1d(stretched_time_vector, np.reshape(vector_rescaled[:,0], -1), kind='cubic')
		Yinterpolation = interp1d(stretched_time_vector, np.reshape(vector_rescaled[:,1], -1), kind='cubic')
		Thetainterpolation = interp1d(stretched_time_vector, np.reshape(vector_rescaled[:,2], -1), kind='cubic')
			
		interpolation_dict = {'inter_x_bee': Xinterpolation,
							  'inter_y_bee': Yinterpolation,
							  'inter_theta_bee': Thetainterpolation,
							  'duration': duration}
			
		return interpolation_dict

	def generate_complete_trajectory(self, nb_waggle, sample_freq=1000):

		def trajectory_merger(trajectory_tab):
    
		    nb_trajectory = len(trajectory_tab)
		    
		    merged_trajectory = trajectory_tab[0]
		        
		    for i in range(1, nb_trajectory):
		        last_position = merged_trajectory[-1]
		        offset_trajectory = trajectory_tab[i]
		        offset_trajectory[:,:2] = offset_trajectory[:,:2] + last_position[:2] 
		        merged_trajectory = np.vstack([merged_trajectory, offset_trajectory])
		     
		    return merged_trajectory

		def return_robot_trajetory(trajectory):
		    exc = self.config['exc_mm']
		    robot_traj = copy.copy(trajectory)
		    robot_traj[:,0] = trajectory[:,0] + exc * np.cos(trajectory[:,2])
		    robot_traj[:,1] = trajectory[:,1] + exc * np.sin(trajectory[:,2])
		    return robot_traj

		def center_trajectory(trajectory, averx=None, avery=None):
		    minx = np.amin(trajectory[:,0])
		    maxx = np.amax(trajectory[:,0])
		    miny = np.amin(trajectory[:,1])
		    maxy = np.amax(trajectory[:,1])
		    
		    if averx is None or avery is None:
		        averx = (maxx + minx)/2
		        avery = (maxy + miny)/2
		    
		    trajectory[:,0] = trajectory[:,0] - averx
		    trajectory[:,1] = trajectory[:,1] - avery
		    
		    return trajectory, averx, avery

		waggle_function = self.waggle_function_list[nb_waggle-1]

		t_return = np.linspace(0, self.return_function['duration'], int(self.return_function['duration']*sample_freq))
		t_waggle = np.linspace(0, waggle_function['duration'], int(waggle_function['duration']*sample_freq))

		self.total_duration = 2 * self.return_function['duration'] + 2 * waggle_function['duration']

		waggle = np.transpose(np.array([waggle_function['inter_x_bee'](t_waggle), 
                                		waggle_function['inter_y_bee'](t_waggle),
                                		waggle_function['inter_theta_bee'](t_waggle)]))

		waggle_bis = copy.copy(waggle)
		waggle_bis[:,1:] = -waggle_bis[:,1:]
		return_loop = np.transpose(np.array([self.return_function['inter_x_bee'](t_return[2:]), 
                                    		 self.return_function['inter_y_bee'](t_return[2:]),
                                     		 self.return_function['inter_theta_bee'](t_return[2:])]))

		return_loop_bis = copy.copy(return_loop)
		return_loop_bis[:,1:] = -return_loop_bis[:,1:]

		tab = [return_loop, waggle, return_loop_bis, waggle_bis]

		self.bee_trajectory = trajectory_merger(tab)

		self.total_step = self.bee_trajectory.shape[0]

		self.addr_return_1 = 0
		self.addr_waggle_1 = self.addr_return_1 + return_loop.shape[0]
		self.addr_return_2 = self.addr_waggle_1 + waggle.shape[0]
		self.addr_waggle_2 = self.addr_return_2 + return_loop.shape[0]

		#Map any angle between -pi and pi
		angle_mapper = lambda x : ( x - np.pi ) % ( 2 * np.pi ) - np.pi
		angle_mapper_vectorize = np.vectorize(angle_mapper)

		self.bee_trajectory[:,2] = angle_mapper_vectorize(self.bee_trajectory[:,2])

		self.robot_trajectory = return_robot_trajetory(self.bee_trajectory)
		self.robot_trajectory, averx, avery = center_trajectory(self.robot_trajectory)
		self.bee_trajectory, _, _ = center_trajectory(self.bee_trajectory, averx, avery)
		self.current_waggle_trajectory = nb_waggle
		self.trajectory_ready = True

	def export_to_svg(self, path='dancepath.svg'):
    
	    def coord_to_string(command, x, y):
	        return command + ' ' + str(x)+','+str(y)
	    
	    dwg = svgwrite.Drawing('dancepath.svg', size=('60mm', '60mm'), viewBox=('-30 -30 30 30'))
	    dwg.add(dwg.line(start=(-30, -30), 
	                     end=(30, -30), 
	                     stroke=svgwrite.rgb(10, 10, 10, '%')))
	    dwg.add(dwg.line(start=(-30, -30), 
	                     end=(-30, 30), 
	                     stroke=svgwrite.rgb(10, 10, 10, '%')))
	    
	    path = dwg.path(d=coord_to_string('M', self.robot_trajectory[0,0], self.robot_trajectory[0,1]))
	    
	    steps = 10
	    for i in range(steps, self.robot_trajectory.shape[0], steps):
	        path.push(coord_to_string('L', self.robot_trajectory[i,0], self.robot_trajectory[i,1]))
	        
	    path.push(coord_to_string('L', self.robot_trajectory[0,0], self.robot_trajectory[0,1]))
	    dwg.add(path)
	    dwg.save()

	def init_trajectory(self, nb_waggle):
		self.generate_complete_trajectory(nb_waggle)
		self.current_position_step = 0

	def get_next_pos(self):
		self.current_position_step = self.current_position_step + 10
		if self.current_position_step >= self.total_step:
			self.current_position_step = self.current_position_step % self.total_step

		return self.bee_trajectory[self.current_position_step, :]


if __name__ == '__main__':
	bee = BeeBoogie()
	bee.init_trajectory(15)

	print(bee.total_duration, bee.total_step, bee.robot_trajectory.shape, bee.bee_trajectory.shape)
	#plt.scatter(bee.robot_trajectory[:,0], bee.robot_trajectory[:,1], c=bee.robot_trajectory[:,2], cmap='magma')
	#plt.scatter(bee.bee_trajectory[:,0], bee.bee_trajectory[:,1], c=bee.bee_trajectory[:,2])
	#plt.show()

	fig, ax = plt.subplots()
	ln, = plt.plot([], [], 'r.', animated=True)

	def init():
	    ax.set_xlim(-30, 30)
	    ax.set_ylim(-30, 30)
	    return ln,

	def update(frame):
	    data = bee.get_next_pos() 
	    ln.set_data(data[0], data[1])
	    return ln,

	print(bee.total_step)
	ani = FuncAnimation(fig, update, frames=bee.total_step-10,
            init_func=init, blit=True, interval=10)
	plt.show()
'''
	import matplotlib.pyplot as plt
	import numpy as np
	from matplotlib.animation import FuncAnimation

	list_var_points = (1, 5, 4, 9, 8, 2, 6, 5, 2, 1, 9, 7, 10)

	fig, ax = plt.subplots()
	xfixdata, yfixdata = [14], [8]
	xdata, ydata = [5], []
	ln, = plt.plot([], [], 'ro', animated=True)
	plt.plot([xfixdata], [yfixdata], 'bo')

	def init():
	    ax.set_xlim(0, 15)
	    ax.set_ylim(0, 15)
	    return ln,

	def update(frame):
	    ydata = list_var_points[frame]
	    ln.set_data(xdata, ydata)
	    return ln,          


	ani = FuncAnimation(fig, update, frames=range(len(list_var_points)),
	            init_func=init, blit=True)
	plt.show()'''