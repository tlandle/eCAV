import matplotlib
import matplotlib.pyplot as plt
import math
import numpy as np
from numpy import dot
import scipy.integrate
import scipy.io
import time
import datetime
import threading
from random import randint, random
import sys
import copy
from sklearn.cluster import KMeans
import logging

logger = logging.getLogger(__name__)

# class Car():
# 	def __init__(self, dt, numlanes, pos_x, pos_lane, velocity):
# 		self.pos_x = pos_x
# 		self.lane = pos_lane
# 		self.v = velocity
# 		self.dt=dt
# 		self.numlanes=numlanes
# 		self.intentions='None'
# 		self.target_velocity = velocity
# 		self.target_lane = None

# 	def car_lane_shift(self,direction):
# 		if direction > 0:
# 			self.lane += 1
# 			self.lane = min(self.lane,self.numlanes)
# 		elif direction < 0:
# 			self.lane -= 1
# 			self.lane = max(self.lane,0)

# 	def car_change_state(self):
# 		self.pos_x = self.pos_x + self.v * self.dt
# 		self.pos_x = self.pos_x % 1000

class Car():
	def __init__(self, dt, numlanes, pos_x, pos_lane, velocity):
		self.pos_x = pos_x
		self.lane = pos_lane
		self.v = velocity
		self.dt=dt
		self.numlanes=numlanes
		self.intentions='None'
		self.target_velocity = velocity
		self.target_lane = None

		self.changed_roads = False
		self.scrolled = False
		self.slice = None

	def car_lane_shift(self,direction):
		if direction > 0:
			self.lane += 1
			self.lane = min(self.lane,self.numlanes)
		elif direction < 0:
			self.lane -= 1
			self.lane = max(self.lane,0)
		else:
			self.lane = self.lane

	def car_change_state(self,map_length=1000):
		self.pos_x = self.pos_x + self.v * self.dt

		if self.pos_x >= map_length:
			self.scrolled = True

		self.pos_x = self.pos_x % map_length

class Traffic():
	def __init__(self, dt, numlanes, numcars, map_length, x_initial=None, y_initial=None, v_initial=None):
		self.numlanes = numlanes
		self.dt=dt
		self.numcars = numcars
		self.cars_on_road = []
		self.time=0
		self.map_length = map_length

		#Utilities for checking lane constraints
		if x_initial is None:
			self.lane_vehicles = np.zeros((self.map_length,self.numlanes))

			numpos = np.linspace(start=0,stop=79,num=80)

			positions_init = 10*np.random.choice(numpos,size=self.numcars,replace=False)

			for i in range(0,numcars):
				self.cars_on_road.append(Car(self.dt,self.numlanes,int(positions_init[i]),np.random.randint(0,numlanes),np.random.randint(15,25)))

		#HARDCODED FOR TEST
		else:
			xpos_array = x_initial #[410, 750, 270, 610, 290, 610, 220, 780, 130, 210, 120, 330, 460, 470, 580, 0]
			ypos_array = y_initial #[0,0,1,0,2, 0, 3, 2, 0, 3, 2, 2, 3, 2, 3, 1]
			v_array = v_initial # [24, 21, 24, 21, 20, 21, 21, 19, 20, 23, 20, 16, 21, 21, 24, 18]

			for i in range(0,numcars):
				self.cars_on_road.append(Car(self.dt,self.numlanes,int(xpos_array[i]),int(ypos_array[i]),v_array[i]))
		#Testing lanechange occupancy check
		# self.cars_on_road[0].lane = 0
		# self.cars_on_road[0].v = 13
		# self.cars_on_road[0].pos_x = 1
		# self.cars_on_road[1].pos_x = 998 #self.cars_on_road[0].pos_x - 20
		# self.cars_on_road[1].v = self.cars_on_road[0].v + 10
		# self.cars_on_road[1].lane = self.cars_on_road[0].lane

		# print(self.numlanes)

		self.update_grid_occupancies()

		self.check_spawn_constraints()

		# self.printstates()

	def respawn_vehicles(self,xcars, ycars, vel):
		for i in range(0,self.numcars):
			self.cars_on_road[i].pos_x = xcars[i]
			self.cars_on_road[i].lane = ycars[i]
			self.cars_on_road[i].v = vel[i]
		self.update_grid_occupancies()
		self.check_spawn_constraints()

	def printstates(self):
		carnum = 0
		for i in self.cars_on_road:
			print("X Coordinate of ", carnum, " Is: ", i.pos_x)
			print("Y Coordinate of ", carnum, " Is: ", i.lane)
			print("Velocity of ", carnum, " Is: ", i.v)
			carnum += 1

	def update_grid_occupancies(self):
		self.lane_vehicles = np.zeros((self.map_length,self.numlanes))
		carnum = 0
		for i in self.cars_on_road:
			self.lane_vehicles[int(i.pos_x),int(i.lane)] = carnum+1
			carnum = carnum + 1

	def check_spawn_constraints(self):
		#At present a bruteforce check for spacing between cars and a crude solution, can be optimized later
		for i in range(0,self.numlanes):
			indices_of_interest = np.nonzero(self.lane_vehicles[:,i])
			indices_of_interest = indices_of_interest[0]
			logger.info("indices of interest:")
			logger.info(indices_of_interest)
			if len(indices_of_interest) <= 1:
				continue
			else:
				for j in range(0,len(indices_of_interest)):
					for k in range(j+1,len(indices_of_interest)):
						distance_between = indices_of_interest[j] - indices_of_interest[k]
						coord_j = int(self.lane_vehicles[indices_of_interest[j],i]-1)
						coord_k = int(self.lane_vehicles[indices_of_interest[k],i]-1)
						if (distance_between < 3) and (distance_between > 0):
							self.cars_on_road[coord_j].pos_x = (self.cars_on_road[coord_k].pos_x+3) % self.map_length
						elif (distance_between > -3) and (distance_between < 0):
							self.cars_on_road[coord_k].pos_x = (self.cars_on_road[coord_k].pos_x+3) % self.map_length
						self.update_grid_occupancies()

	def check_adjacent_occupancies(self,car):

		#TODO Rework ahead check to account for overspill/scrolling

		margin_safety = 15
		rear_ind = int(car.pos_x-min(margin_safety,car.pos_x))
		forward_ind = int(min(car.pos_x+margin_safety,self.map_length-1))

		if car.lane < self.numlanes - 1:
			grid_slice_upper = self.lane_vehicles[rear_ind:forward_ind,car.lane+1]
			if np.sum(grid_slice_upper) > 0:
				occupied_above = 1
			else:
				occupied_above = 0

			if (car.pos_x+margin_safety) > self.map_length-1:
				forward_ind_ahead = int((car.pos_x+margin_safety)%self.map_length)
				grid_slice_upper = self.lane_vehicles[0:forward_ind_ahead+1,car.lane+1]
				if np.sum(grid_slice_upper) > 0:
					occupied_above = 1

			if car.pos_x < margin_safety:
				rear_ind = int(self.map_length-(margin_safety-car.pos_x))
				grid_slice_upper = self.lane_vehicles[rear_ind:self.map_length-1,car.lane+1]
				if np.sum(grid_slice_upper) > 0:
					occupied_above = 1

		elif car.lane == self.numlanes - 1:
			occupied_above = 2

		if car.lane > 0:
			grid_slice_lower = self.lane_vehicles[rear_ind:forward_ind,car.lane-1]

			if np.sum(grid_slice_lower) > 0:
				occupied_below = 1
			else:
				occupied_below = 0

			if (car.pos_x+margin_safety) > self.map_length-1:
				forward_ind_ahead = int((car.pos_x+margin_safety)%self.map_length)
				grid_slice_lower = self.lane_vehicles[0:forward_ind_ahead+1,car.lane-1]
				if np.sum(grid_slice_lower) > 0:
					occupied_below = 1

			if car.pos_x < margin_safety:
				rear_ind = int(self.map_length-(margin_safety-car.pos_x))
				grid_slice_lower = self.lane_vehicles[rear_ind:self.map_length-1,car.lane-1]
				if np.sum(grid_slice_lower) > 0:
					occupied_below = 1

		elif car.lane == 0:
			occupied_below = 2

		return occupied_below, occupied_above

	def check_ahead(self,car,margin_ahead=15):

		#Checking ahead until map edge
		#Rework later, pin in it for now

		#margin_ahead = 11
		indice_ahead = int(min(car.pos_x+margin_ahead,self.map_length-1))

		grid_slice_required =self.lane_vehicles[int(car.pos_x+1):indice_ahead,int(car.lane)]
		vehicle_ahead = None

		for i in range(0,len(grid_slice_required)):
			if grid_slice_required[i] == 0:
				continue
			else:
				vehicle_ahead = int(grid_slice_required[i]-1)
				break

		if (car.pos_x + margin_ahead > self.map_length-1) and (vehicle_ahead is None):
			indice_ahead = int((car.pos_x+margin_ahead)%self.map_length)
			grid_slice_required = self.lane_vehicles[0:indice_ahead+1,int(car.lane)]

			for i in range(0,len(grid_slice_required)):
				if grid_slice_required[i] == 0:
					continue
				else:
					vehicle_ahead = int(grid_slice_required[i]-1)
					break

		# print("Vehicle Ahead: ", vehicle_ahead)

		return vehicle_ahead

	def time_tick(self,mode='Auto'):
		margin_safety = 10

		if mode == 'Graph':
			for cars in self.cars_on_road:
				vehicle_ahead = self.check_ahead(cars,10)

				if cars.intentions == 'Lane Change 1':
					print("Lane Changed")
					print("Vehicle: ", cars.pos_x)
					cars.car_lane_shift(1)
					cars.intentions = 'None'
					#cars.car_change_state(self.map_length)
				elif cars.intentions == 'Lane Change -1':
					cars.car_lane_shift(-1)
					print("Lane Changed Down")
					print("Vehicle: ", cars.pos_x)
					cars.intentions = 'None'
					#cars.car_change_state(self.map_length)
				else:
					#print("Other Lane Occupied")
					if vehicle_ahead is not None:
						if cars.v > self.cars_on_road[vehicle_ahead].v:
							cars.v = self.cars_on_road[vehicle_ahead].v
					else: #This has been added on 11/09/2021
						if cars.v != cars.target_velocity:# and mode=='Manual':
							cars.v = cars.target_velocity
					cars.car_change_state(self.map_length)
			self.update_grid_occupancies()
			self.time += self.dt
		else:
			for cars in self.cars_on_road:
				occupied_below, occupied_above = self.check_adjacent_occupancies(cars)

				vehicle_ahead = self.check_ahead(cars)

				if cars.target_lane is None and mode == 'Manual':
					if vehicle_ahead is not None:
						if cars.v > self.cars_on_road[vehicle_ahead].v:
							cars.v = self.cars_on_road[vehicle_ahead].v
							if cars.lane < self.numlanes - 1:
								cars.intentions = 'Lane Change 1'
							else:
								cars.intentions = 'Lane Change -1'
					else:
						if cars.v != cars.target_velocity:# and mode=='Manual':
							cars.v = cars.target_velocity
				else:
					if cars.target_lane is not None:
						if cars.target_lane > cars.lane:
							cars.intentions = 'Lane Change 1'
						elif cars.target_lane < cars.lane:
							cars.intentions = 'Lane Change -1'
						else:
							cars.intentions = 'None'

						if vehicle_ahead is not None:
							if cars.v > self.cars_on_road[vehicle_ahead].v:
								cars.v = self.cars_on_road[vehicle_ahead].v
						else:
							if cars.v != cars.target_velocity:# and mode=='Manual':
								cars.v = cars.target_velocity

				if cars.intentions == 'None':
					cars.car_change_state(self.map_length)
				elif cars.intentions == 'Lane Change 1' and (occupied_above == 0):
					print("Lane Changed")
					print("Vehicle: ", cars.pos_x)
					cars.car_lane_shift(1)
					cars.intentions = 'None'
				elif cars.intentions == 'Lane Change -1' and (occupied_below == 0):
					cars.car_lane_shift(-1)
					print("Lane Changed Down")
					print("Vehicle: ", cars.pos_x)
					cars.intentions = 'None'
				else:
					#print("Other Lane Occupied")
					cars.car_change_state(self.map_length)

			self.update_grid_occupancies()

			self.time += self.dt

	def ret_car_locations(self):
		x_array = np.zeros((self.numcars,1))
		y_array = np.zeros((self.numcars,1))
		target_velocities = np.zeros((self.numcars,1))
		velocities = np.zeros((self.numcars,1))
		carnum = 0
		for car in self.cars_on_road:
			x_array[carnum,0] = car.pos_x
			y_array[carnum,0] = car.lane
			target_velocities[carnum,0] = car.target_velocity
			velocities[carnum,0] = car.v
			carnum += 1
		return x_array, y_array, target_velocities, velocities

	def check_collisions(self):
		# collision_distance = 2
		# collisions_rec = np.zeros((1,self.numcars))

		# for i in range(0,self.numcars):
		# 	vehicle_ahead = self.check_ahead(self.cars_on_road[i],margin_ahead=collision_distance)
		# 	if vehicle_ahead is not None:
		# 		collisions_rec[0,i] = 1
		# 		self.printstates()
		# 		x_states, y_states = self.ret_car_locations()
		# return collisions_rec
		collision_distance = 2
		collisions_rec = np.zeros((self.numcars,self.numcars))

		for i in range(0,self.numlanes):
			vehicle_indices = self.lane_vehicles[np.where(self.lane_vehicles[:,i] != 0),i]
			vehicle_indices = vehicle_indices[0].astype(int)

			for j in range(0,len(vehicle_indices)):
				vehicle_j_pos = self.cars_on_road[vehicle_indices[j]-1].pos_x
				for k in range(j+1,len(vehicle_indices)):
					vehicle_k_pos = self.cars_on_road[vehicle_indices[k]-1].pos_x

					if vehicle_j_pos <= collision_distance:
						distance = (vehicle_k_pos+collision_distance)%self.map_length - vehicle_j_pos
					elif vehicle_j_pos >= self.map_length-collision_distance:
						distance = (vehicle_j_pos+collision_distance)%self.map_length - vehicle_k_pos
					else:
						distance = vehicle_j_pos - vehicle_k_pos

					# print("Distance: ", distance)
					if abs(distance) <= collision_distance:
						collisions_rec[vehicle_indices[j]-1,vehicle_indices[k]-1] = 1
						collisions_rec[vehicle_indices[k]-1,vehicle_indices[j]-1] = 1

		return collisions_rec

def main():
	print(__file__ + " start!!")

	map_length = 1000
	discretization_size=1
	numlanes = 4
	numcars = 24
	maxtime = 50

	spawn_lane = np.random.randint(low=0,high=numlanes)

	spawn_x = np.load('sim_configs/positions_init.npy')
	spawn_y = np.load('sim_configs/lanes_init.npy')
	spawn_v = np.load('sim_configs/vel_init.npy')

	Traffic_Tracker = Traffic(0.2,numlanes,numcars,map_length,x_initial=spawn_x,y_initial=spawn_y,v_initial=spawn_v)

	xcars = np.empty((numcars,0))
	ycars = np.empty((numcars,0))
	target_velocities = np.empty((numcars,0))
	velocities = np.empty((numcars,0))

	x_states, y_states, tv, v = Traffic_Tracker.ret_car_locations()
	x_initial, y_initial, v_initial = x_states, y_states, v

	xcars = np.hstack((xcars, x_states))
	ycars = np.hstack((ycars, y_states))
	target_velocities = np.hstack((target_velocities,tv))
	velocities = np.hstack((velocities,v))

	# sys.exit()

	time = 0
	lane_change_flag = 0

	while Traffic_Tracker.time < maxtime:
		Traffic_Tracker.time_tick()
		x_states, y_states, tv, v = Traffic_Tracker.ret_car_locations()
		xcars = np.hstack((xcars, x_states))
		ycars = np.hstack((ycars, y_states))
		target_velocities = np.hstack((target_velocities,tv))
		velocities = np.hstack((velocities,v))
		# if Traffic_Tracker.time >= 5:
		# 	Traffic_Tracker.cars_on_road[0].target_lane = 3
		# 	Traffic_Tracker.cars_on_road[0].pos_x = Traffic_Tracker.cars_on_road[1].pos_x
		# 	Traffic_Tracker.cars_on_road[0].lane = Traffic_Tracker.cars_on_road[1].lane

		#Getting velocities list
		velocities_list = []
		for cars in Traffic_Tracker.cars_on_road:
			velocities_list.append(cars.target_velocity)

		#K-Means clustering
		kmeans = KMeans(n_clusters = numlanes, random_state=0).fit_predict(np.array(velocities_list).reshape(-1,1))
		print("Cluster Indices: ", kmeans)

		#Assigning target lanes
		j=0
		for cars in Traffic_Tracker.cars_on_road:
			cars.target_lane = kmeans[j]
			j += 1

		#Clustering Ends##

		collisions_rec = Traffic_Tracker.check_collisions()
		if np.sum(collisions_rec) > 0:
			print("Collided: ", collisions_rec)
			print("Time: ", Traffic_Tracker.time)
			Traffic_Tracker.printstates()
			sys.exit()

		plot_name = '/home/chandramouli/Documents/collab-sandbox/comparison_three_test/clustering/' + 'snapshot' + str(Traffic_Tracker.time/0.2) + '.jpg'

		plt.cla()
		# for i in range(0,Traffic_Tracker.numcars):
			# plt.plot(xcars[i,:],ycars[i,:])
		plt.scatter(x_states,y_states,marker='o',s=20)
		plt.xlim([0,1000])
			# print(plot_name)
		plt.savefig(plot_name)
		plt.pause(0.001)

	print("Terminated Successfully")

	#Getting lanes
	lanes_list = []
	for cars in Traffic_Tracker.cars_on_road:
		lanes_list.append(cars.lane)

	print("Final Lane: ", lanes_list)
	scipy.io.savemat("/home/chandramouli/Documents/collab-sandbox/comparison_three_test/clustering/States_x_clustering.mat",mdict={'X':xcars})
	scipy.io.savemat("/home/chandramouli/Documents/collab-sandbox/comparison_three_test/clustering/States_y_clustering.mat",mdict={'Y':ycars})
	scipy.io.savemat("/home/chandramouli/Documents/collab-sandbox/comparison_three_test/clustering/States_v_clustering.mat",mdict={'V':velocities})
	scipy.io.savemat("/home/chandramouli/Documents/collab-sandbox/comparison_three_test/clustering/States_tv_clustering.mat",mdict={'TV':target_velocities})

#################################SECOND SIMULATION, NO CLUSTERING#########################################################
	Traffic_Tracker = Traffic(0.2,numlanes,numcars,map_length,x_initial.ravel(), y_initial.ravel(), v_initial.ravel())

	xcars = np.empty((numcars,0))
	ycars = np.empty((numcars,0))
	target_velocities = np.empty((numcars,0))
	velocities = np.empty((numcars,0))

	x_states, y_states, tv, v = Traffic_Tracker.ret_car_locations()

	xcars = np.hstack((xcars, x_states))
	ycars = np.hstack((ycars, y_states))
	target_velocities = np.hstack((target_velocities,tv))
	velocities = np.hstack((velocities,v))

	time = 0
	lane_change_flag = 0

	while Traffic_Tracker.time < maxtime:
		Traffic_Tracker.time_tick(mode='Manual')
		x_states, y_states, tv, v = Traffic_Tracker.ret_car_locations()
		xcars = np.hstack((xcars, x_states))
		ycars = np.hstack((ycars, y_states))
		target_velocities = np.hstack((target_velocities,tv))
		velocities = np.hstack((velocities,v))

		collisions_rec = Traffic_Tracker.check_collisions()
		if np.sum(collisions_rec) > 0:
			print("Collided: ", collisions_rec)
			print("Time: ", Traffic_Tracker.time)
			Traffic_Tracker.printstates()
			sys.exit()

		plot_name = '/home/chandramouli/Documents/collab-sandbox/comparison_three_test/greedy/' + 'snapshot' + str(Traffic_Tracker.time/0.2) + '.jpg'

		plt.cla()
		#for i in range(0,Traffic_Tracker.numcars):
			# plt.plot(xcars[i,:],ycars[i,:])
		plt.scatter(x_states,y_states,marker='o',s=20)
			# print(plot_name)
		plt.xlim([0,1000])
		plt.ylim([-1,5])
		plt.savefig(plot_name)

		plt.pause(0.001)

	print("Terminated Successfully")

	#Getting lanes
	lanes_list = []
	for cars in Traffic_Tracker.cars_on_road:
		lanes_list.append(cars.lane)

	print("Final Lane: ", lanes_list)
	scipy.io.savemat("/home/chandramouli/Documents/collab-sandbox/comparison_three_test/greedy/States_x_no_clustering.mat",mdict={'Xn':xcars})
	scipy.io.savemat("/home/chandramouli/Documents/collab-sandbox/comparison_three_test/greedy/States_y_no_clustering.mat",mdict={'Yn':ycars})
	scipy.io.savemat("/home/chandramouli/Documents/collab-sandbox/comparison_three_test/greedy/States_v_no_clustering.mat",mdict={'Vn':velocities})
	scipy.io.savemat("/home/chandramouli/Documents/collab-sandbox/comparison_three_test/greedy/States_tv_no_clustering.mat",mdict={'TVn':target_velocities})

if __name__ == '__main__':
    main()
