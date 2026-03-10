import numpy as np

class detectOutlier:
    counter = 0
    firstFill = False

    def __init__(self, arrlen, sigmamult): #Initialize array length and sigma multiplier variable for outlier detection
        self.sigmamult = sigmamult
        self.arrlen = arrlen
        self.arr = np.array([])
 
    def add(self, value): # Adds a number to the class's array and checks if it is an outlier
        if self.arr.size < self.arrlen and self.firstFill is False:
            self.arr = np.append(self.arr, value)
            return True
        else:
            firstFill = True
            mean = np.mean(self.arr)
            std = np.std(self.arr)

            if (value < mean + self.sigmamult * std) and (value > mean - self.sigmamult * std):
                if (self.counter > self.arrlen-1): # Ring Buffering
                    self.counter = 0
                self.arr[self.counter] = value
                self.counter+=1
                print(f"array: {self.arr} , mean: {mean}, std: {std}")
                return True
            else:
                print("disregarded number...")
                return False
        
            

    
