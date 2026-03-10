import matplotlib.pyplot as plt
import numpy as np
import csv
import os

set_num = 2  # refers to media 
test_name = "axial_load_tests"
trial_num = 1 # for florian: this is trial
parent_folder = "/home/myeoh/Documents/GitHub/arcsnake_v2/tests/ScrewTestScripts/data_files"


csv_to_read = os.path.join(parent_folder, "")
with open('eggs.csv', newline='') as csvfile:
