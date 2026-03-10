from os.path import dirname, realpath  
import sys
from charset_normalizer import detect  
arcsnake_v2_path = dirname(dirname(realpath(__file__)))  
sys.path.append(arcsnake_v2_path)  

import pandas as pd
import matplotlib.pyplot as plt
from core.outlier_detection import detectOutlier

filter = detectOutlier(75, 3)
filtered_column = []
lastResult = 0

df = pd.read_csv(r"tests\ScrewTestScripts\data_files\6_22_test3.csv")
saved_column = df['linear speed'] #you can also use df['column_name']
print(saved_column)

plt.subplots(2)
plt.plot(saved_column)
plt.ylabel('linear speed')
plt.xlabel('Line')
plt.subplot(2,1,1)

for i in saved_column:
    if (filter.add(i)):
        lastResult = i
        filtered_column.append(i)
    else:
        filtered_column.append(lastResult)

plt.plot(filtered_column)
plt.ylabel('Filtered linear speed')
plt.xlabel('Line')
plt.subplot(2,1,2)


plt.show()