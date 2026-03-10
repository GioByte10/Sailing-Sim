import cv2 as cv
import numpy as np
import csv
from scipy.signal import butter, lfilter
from matplotlib import pyplot as plt
import math
from sklearn.linear_model import LinearRegression

### Reads in data from csv file with format from FT sensor log
def read_sensor_csv(fname):

	with open(fname) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=',')
		csv_list = list(csv_reader)
		ft_data = np.empty((len(csv_list)-7, 6))
		line_count = 0
		csv_reader = iter(csv_reader)
		for row in csv_list:
			# print(line_count)
			if line_count > 6:
				ft_data[line_count-7, :] = row[3:9]
			line_count += 1

	return ft_data


### Reads in data from motor csv files
def read_motor_csv(fname):

	with open(fname) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=',')
		csv_list = list(csv_reader)
		data = np.empty((len(csv_list)-1, 4))
		line_count = 0
		for row in csv_list:
			if line_count != 0:
				data[line_count-1,:] = row
			line_count += 1

	return data


### Reads in data from motor csv files
def read_tr_motor_csv(fname):

	with open(fname) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=',')
		csv_list = list(csv_reader)
		data = np.empty((len(csv_list)-1, 6))
		line_count = 0
		for row in csv_list:
			if line_count != 0:
				data[line_count-1, :] = row
			line_count += 1

	return data


### Peforms butterworth LPF on data
def butter_lpf(data, cutoff, fs, order):
	nyq = 0.5 * fs
	normal_cutoff = cutoff / nyq
	b, a = butter(order, normal_cutoff, btype='low', analog=False)
	filtered_data = lfilter(b, a, data)
	return filtered_data


### Filter motor data
def filter_motor_data(data, cutoff, fs, order, tr=False):
	w = data[:,1]
	t = data[:,2]
	v = data[:,3]
	if tr==True:
		p = data[:,4]
	filtered_data = np.empty(np.shape(data))
	filtered_data[:,0] = data[:,0]
	nyq = 0.5 * fs
	normal_cutoff = cutoff / nyq
	b, a = butter(order, normal_cutoff, btype='low', analog=False)
	filtered_data[:,1] = lfilter(b, a, w)
	filtered_data[:, 2] = lfilter(b, a, t)
	filtered_data[:, 3] = lfilter(b, a, v)
	if tr==True:
		filtered_data[:, 4] = lfilter(b, a, p)

	return filtered_data


### Filter ft data
def filter_ft_data(data, cutoff, fs, order):


	filtered_data = np.empty(np.shape(data))
	nyq = 0.5 * fs
	normal_cutoff = cutoff / nyq
	b, a = butter(order, normal_cutoff, btype='low', analog=False)
	filtered_data[:, 0] = lfilter(b, a, data[:,0])
	filtered_data[:, 1] = lfilter(b, a, data[:,1])
	filtered_data[:, 2] = lfilter(b, a, data[:,2])
	filtered_data[:, 3] = lfilter(b, a, data[:, 3])
	filtered_data[:, 4] = lfilter(b, a, data[:, 4])
	filtered_data[:, 5] = lfilter(b, a, data[:, 5])

	return filtered_data


### Plots FT sensor data
def plot_ft_data(ft_data, a=0, b=None, fname=None, peaks=[], sync=False):

	if sync == False:
		if b == None:
			b = len(ft_data)
		plt.figure(figsize=(16, 10))
		plt.plot(ft_data[a:b,0])
		plt.plot(ft_data[a:b,1])
		plt.plot(ft_data[a:b,2])
		plt.plot(ft_data[a:b, 3])
		plt.plot(ft_data[a:b, 4])
		plt.plot(ft_data[a:b, 5])
		for i in range(len(peaks)):
			plt.axvline(peaks[i])
		plt.legend(['fx', 'fy', 'fz', 'tx', 'ty', 'tz'])
		if fname != None:
			plt.savefig(fname)
		plt.show(block=False)


	if sync == True:
		plt.figure(figsize=(16, 10))
		plt.plot(ft_data[:1000, 5])
		plt.show(block=False)

### Plot motor data
def plot_motor_data(data, fname, tr=False):

	plt.figure(figsize=(16, 10))
	# plt.plot(data[:,0])
	plt.plot(data[:,1])
	plt.plot(data[:,2])
	if tr == False:
		plt.plot(data[:,3])
		plt.legend(['angular speed', 'torque', 'linear speed'])
	if tr == True:
		plt.plot(data[:,4])
		plt.legend(['angular speed', 'torque', 'angular position'])
		
	if fname != None:
		plt.savefig(fname)
	plt.show(block=False)


### Plots all data together
def plot_data(fname, ft_data, motor_data, a, b, c, d, e, f, tr=False):
	
	if tr==False:
		plt.figure(figsize=(16, 10))
		ax1 = plt.subplot(234)
		ax1.plot(motor_data[:,0], motor_data[:, 1])
		ax1.plot(motor_data[:, 0], motor_data[:, 2])
		ax1.plot(motor_data[:, 0], motor_data[:, 3])
		ax1.legend(['angular speed', 'torque', 'linear speed'])


		ax2 = plt.subplot(235)
		ax2.plot(motor_data[a:b, 0], motor_data[a:b, 1])
		ax2.plot(motor_data[a:b, 0], motor_data[a:b, 2])
		ax2.plot(motor_data[a:b, 0], motor_data[a:b, 3])
		ax2.legend(['angular speed', 'torque', 'linear speed'])

		ax3 = plt.subplot(231)
		ax3.plot(ft_data[:, 0])
		ax3.plot(ft_data[:, 1])
		ax3.plot(ft_data[:, 2])
		ax3.plot(ft_data[:, 3])
		ax3.plot(ft_data[:, 4])
		ax3.plot(ft_data[:, 5])
		ax3.legend(['fx', 'fy', 'fz', 'tx', 'ty', 'tz'])

		ax4 = plt.subplot(232)
		ax4.plot(ft_data[c:d, 0])
		ax4.plot(ft_data[c:d, 1])
		ax4.plot(ft_data[c:d, 2])
		ax4.plot(ft_data[c:d, 3])
		ax4.plot(ft_data[c:d, 4])
		ax4.plot(ft_data[c:d, 5])
		ax4.legend(['fx', 'fy', 'fz', 'tx', 'ty', 'tz'])

		ax5 = plt.subplot(233)
		ax5.plot(ft_data[e:f, 0])
		ax5.plot(ft_data[e:f, 1])
		ax5.plot(ft_data[e:f, 2])
		ax5.plot(ft_data[e:f, 3])
		ax5.plot(ft_data[e:f, 4])
		ax5.plot(ft_data[e:f, 5])
		ax5.legend(['fx', 'fy', 'fz', 'tx', 'ty', 'tz'])

	if tr==True:

		plt.figure(figsize=(16, 10))
		ax1 = plt.subplot(211)
		ax1.plot(motor_data[3:, 0], motor_data[3:, 1])
		ax1.plot(motor_data[3:, 0], motor_data[3:, 2])
		# ax1.plot(motor_data[3:, 0], motor_data[3:, 3])
		ax1.plot(motor_data[3:, 0], motor_data[3:, 4])
		ax1.legend(['angular speed', 'torque', 'angular position'])

		ax4 = plt.subplot(212)
		ax4.plot(ft_data[c:d, 0])
		ax4.plot(ft_data[c:d, 1])
		ax4.plot(ft_data[c:d, 2])
		ax4.plot(ft_data[c:d, 3])
		ax4.plot(ft_data[c:d, 4])
		ax4.plot(ft_data[c:d, 5])
		ax4.legend(['fx', 'fy', 'fz', 'tx', 'ty', 'tz'])
		

	if fname != None:
		plt.savefig(fname)

	plt.show(block=False)


### Plots motor data and ft data side by side
def plot_both_data(motor_data, ft_data, fname=None):
	
	plt.figure(figsize=(10, 4))
	ax1 = plt.subplot(121)
	ax1.plot(motor_data[:,0], motor_data[:, 1])
	ax1.plot(motor_data[:,0], motor_data[:, 2])
	ax1.plot(motor_data[:,0], motor_data[:, 3])
	ax1.legend(['angular speed', 'torque', 'linear speed'])
	ax1.set_title('Motor/Encoder Data')
	# ax1.xlabel('time')
	# ax1.ylabel('rad/s')

	ax2 = plt.subplot(122)
	ax2.plot(ft_data[:, 0])
	ax2.plot(ft_data[:, 1])
	ax2.plot(ft_data[:, 2])
	ax2.plot(ft_data[:, 3])
	ax2.plot(ft_data[:, 4])
	ax2.plot(ft_data[:, 5])
	ax2.legend(['fx', 'fy', 'fz', 'tx', 'ty', 'tz'], loc='lower right')
	# ax2.set_ylim(-15, 10)
	# ax2.set_yticks(np.arange(-15,10,2.5))
	ax2.set_title('Sensor Data')
	
	if fname != None:
		plt.savefig(fname)

	plt.show(block=False)


### function plots traveling velocity vs. Angular Velocity
def plot_vel(x, set_y, err, legend, fname):
	for i in range(len(set_y)):
		if i == 0:
			plt.errorbar(x, set_y[i,:], marker='o')
		else:
			plt.errorbar(x, set_y[i,:], yerr=err[i-1,:], marker='o', capsize=6)

	plt.xlabel('Angular Velocity')
	plt.ylabel('Traveling Linear Velocity')
	plt.legend(legend)
	plt.savefig(fname)
	plt.show(block=False)


### function plots thrust force vs. Angular Velocity
def plot_thrust(x, set_y, err, legend, fname):
	for i in range(len(set_y)):
		plt.errorbar(x, set_y[i,:], yerr=err[i:], marker='o', capsize=6)

	plt.xlabel('Angular Velocity')
	plt.ylabel('Thrust Force')
	plt.legend(legend)
	plt.savefig(fname)
	plt.show(block=False)


### plot power efficiency vs. Angular Velocity
def plot_P_eff(x, set_y, legend, fname):
	for i in range(len(set_y)):
		plt.plot(x, set_y[i,:], marker='o')

	plt.xlabel('Angular Velocity')
	plt.ylabel('Power Efficiency')
	plt.legend(legend)
	plt.savefig(fname)
	plt.show(block=False)

### Plots slip vs. Angular Velocity
def plot_slip(x, set_y, err, legend, fname):
	for i in range(len(set_y)):
		plt.errorbar(x, set_y[i,:], yerr=err[i,:], marker='o', capsize=6)

	plt.xlabel('Angular Velocity')
	plt.ylabel('Slip')
	plt.legend(legend)
	plt.savefig(fname)
	plt.show(block=False)


### Plot Cost of transport vs. Angular Velocity
def plot_COT(x, set_y, legend, fname):
	for i in range(len(set_y)):
		plt.plot(x, set_y[i,:], marker='o')

	plt.xlabel('Angular Velocity')
	plt.ylabel('Cost of Transport')
	plt.legend(legend)
	plt.savefig(fname)
	plt.show(block=False)


### Plot thrust force vs. torque for static test
def plot_thrust_vs_torque(set_x, set_y, legend, fname, std=[]):

	lead = 0.147
	lg1 = LinearRegression()
	lg2 = LinearRegression()
	model1 = lg1.fit(set_x[0,:].reshape((-1, 1)), set_y[0,:].reshape((-1, 1)))
	model2 = lg2.fit(set_x[1, :].reshape((-1, 1)), set_y[1, :].reshape((-1, 1)))
	x = np.linspace(0, 2, 100)
	print(model1.coef_, model1.intercept_)
	print(model2.coef_, model2.intercept_)
	y1 = (model1.coef_ * x + model1.intercept_).reshape(-1)
	y2 = (model2.coef_ * x + model2.intercept_).reshape(-1)
	set1_eff = model1.coef_ * lead / (2*np.pi)
	set2_eff = model2.coef_ * lead / (2*np.pi)
	print(set1_eff, set2_eff)
		
	r = 0.0775 # m
	alpha = 16
	T = np.arange(0, 2.0, 0.1)
	ideal_thrust = T / (np.tan(math.radians(alpha)) * r)
	halfeff_thrust = 0.5 * ideal_thrust
	plt.plot(T, ideal_thrust, linestyle='--')
	plt.plot(T, 0.5 * ideal_thrust, linestyle='--')
	plt.plot(T, 0.25 * ideal_thrust, linestyle='--')
	legend.insert(0, '25% Efficiency')
	legend.insert(0, '50% Efficiency')
	legend.insert(0, '100% Efficiency')
	for i in range(len(set_y)):
		plt.plot(set_x[i,:], set_y[i,:], marker='o')
	if len(std) == 0:
		plt.plot(x, y1, color='black')
		plt.plot(x, y2, color='black')
	else:
		for i in range(len(set_y)):
			plt.errorbar(set_x[i, :], set_y[i, :],
			             yerr=std[i, :], marker='o', capsize=6)
	plt.xlabel('Input Torque')
	plt.ylabel('Peak Thrust Forcce')
	plt.legend(legend)
	plt.savefig(fname)
	plt.show(block=False)

	plt.figure()
	plt.errorbar(['Rocks', 'Sand'], [model1.coef_[0][0],
	             model2.coef_[0][0]], marker='o', capsize=6, linestyle='')
	plt.savefig('MA.png')
	plt.show(block=False)



### Plot sensor torque vs. input torque
def plot_sensor_torque(set_x, set_y, legend, fname):
	for i in range(len(set_y)):
		plt.plot(set_x[i, :], set_y[i, :], marker='o')

	plt.plot([0.5, 1.5], [0.5, 1.5])
	plt.xlabel('Input Torque')
	plt.ylabel('Measured Sensor Torque')
	plt.legend(legend)
	plt.savefig(fname)
	plt.show(block=False)


### Plot motor torque for torque ramp test
def plot_motor_torque(set_x, set_y, legend, fname):
	for i in range(len(set_y)):
		plt.plot(set_x[i], set_y[i])

	plt.xlabel('Time (s)')
	plt.ylabel('Input Torque')
	plt.legend(legend)
	plt.savefig(fname)
	plt.show(block=False)


def plot_motor_angpos(set_x, set_y, legend, fname):
	y = set_y[0]
	ddy = np.gradient(np.gradient(y))
	infls = np.where(np.diff(np.sign(ddy)))[0]
	plt.plot(y)
	print(infls)
	plt.show(block=False)


# def find_slip_points(data):


### plot portion of trial for synchronization procedure