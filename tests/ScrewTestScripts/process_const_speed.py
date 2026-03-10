import cv2 as cv
import numpy as np
import csv
from pickle_mixin import test
from scipy.signal import butter, lfilter
from matplotlib import pyplot as plt
from analysis import read_sensor_csv
from utils import *
from utils import filter_motor_data


if __name__ == "__main__":

	just_visualizing = False  # Set true if you just want to visualize the data

	set_num = 1
	test_num = 1

	ft_data = read_sensor_csv('fts_data_files/const_speed_tests/set{0}/test{1}.csv'.format(set_num, test_num))
	motor_data = read_motor_csv('motor_data_files/const_speed_tests/set{0}/test{1}.csv'.format(set_num, test_num))

	filt_motor_data = filter_motor_data(motor_data, cutoff=6, fs=125, order=2)
	filt_ft_data = filter_ft_data(ft_data, cutoff=6, fs=125, order=2)

	if just_visualizing==True:
		plot_motor_data(filt_motor_data, None)
		plot_ft_data(filt_ft_data, 0, None, None)
		plt.show()
	
	elif just_visualizing==False:
		plot_ft_data(filt_ft_data, 0, None, None)

		free_hang_start = int(input('Enter free hang starting index: ')) # Almost always can put 200-300 for these
		free_hang_end = int(input('Enter free hang ending index: '))

		set_down_start = int(input('Enter set down starting index: '))
		set_down_end = int(input('Enter set down ending index: '))

		ft_start = int(input('Enter trial starting index: '))
		ft_end = int(input('Enter trial ending index: '))

		ft_ss_start = int(input('Enter steady state starting index: '))
		ft_ss_end = int(input('Enter steady state ending index: '))

		plot_motor_data(filt_motor_data, None)

		motor_ss_start = int(input('Enter motor steady state starting index: '))
		motor_ss_end = int(input('Enter motor steady state ending index: '))

		figfname = 'figures/const_speed_tests/set{0}/test{1}'.format(set_num, test_num)
		datafname = 'processed_data_files/const_speed_tests/set{0}/testtest{1}'.format(set_num, test_num)

		plot_data(figfname, filt_ft_data, filt_motor_data, motor_ss_start, motor_ss_end, ft_start, ft_end, ft_ss_start, ft_ss_end)

		np.savez(datafname, motor_data=motor_data, filt_motor_data=filt_motor_data, \
			ft_data=ft_data, filt_ft_data=filt_ft_data, ft_start=ft_start, ft_end=ft_end, \
			ft_ss_start=ft_ss_start, ft_ss_end=ft_ss_end, \
			motor_ss_start=motor_ss_start, motor_ss_end=motor_ss_end, \
			free_hang_start=free_hang_start, free_hang_end=free_hang_end, \
			set_down_start=set_down_start, set_down_end=set_down_end)

	plt.show()