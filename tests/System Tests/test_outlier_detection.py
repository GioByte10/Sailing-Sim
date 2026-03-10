from os.path import dirname, realpath  
import sys  
arcsnake_v2_path = dirname(dirname(realpath(__file__)))  
sys.path.append(arcsnake_v2_path)  

from core.outlier_detection import detectOutlier

test = detectOutlier(5, 2)

try:
    while True: 
        print("Enter value:")
        test.add(int(input()))
except(KeyboardInterrupt) as e:
    print(e)

print("ended")