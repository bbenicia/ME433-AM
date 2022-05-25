#print("Hello World!")
from ulab import numpy as np # to get access to ulab numpy functions
# Declare an array with some made up data like
import time 
#import matplotlib.pyplot as plt

swaves=np.zeros(1024)
pi = 3.1415 
f1 = 1 
f2 = 4
f3 = 6 
t = np.linspace ( 0 , 10 , 1024)

# Test some stats functions, like

wave1 = np.sin(2*pi*f1*t)
wave2 = np.sin(2*pi*f2*t)
wave3 = np.sin(2*pi*f3*t)

swaves= wave1 + wave2 + wave3

four= np.fft.fft(swaves)


#plt.plot(time, wave1, label='wave 1')
#plt.plot(time, wave2, label='wave 2')
#plt.plot(time, wave3, label='wave 3')
#plt.plot(time, four, label='fft')

for i in four[0]:
	print((i,))
	time.sleep(0.01)
