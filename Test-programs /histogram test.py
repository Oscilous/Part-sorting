import numpy as np
import RPi.GPIO as GPIO
import time
import matplotlib.pyplot as plt   
   
histArray = np.loadtxt('./Otsu.txt')
maxV = np.amax(histArray)
print(histArray.size)
print(maxV)
#plt.close('all')
#plt.ion()

    
fig, ax = plt.subplots()
bins=[0,0.001,0.002,0.003,0.004,0.005,0.006,0.007,0.008,0.009,0.01,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.09,0.1,0.12,0.14,0.16,0.18,0.2,0.25,0.3,1,2,3,4,5,20]
ax.hist(histArray, bins , density=True,histtype='bar', rwidth=0.8)
ax.set(xlim=(0, 0.5), xticks=np.arange(0, 0.5,0.05),
   ylim=(0, 20), yticks=np.linspace(0, 20, 20))
#plt.xlabel('Smarts')
plt.ylabel('Probability')
plt.subplots_adjust(left=0.15,bottom=0.15)
plt.title(r'Histogram of Otsu values')
#plt.xscale("log")
plt.show()
plt.pause(0.1)