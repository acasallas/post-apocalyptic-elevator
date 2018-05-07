import numpy as np
from scipy.fftpack import fft, ifft
import matplotlib.pyplot as plt
from sys import argv

start_up = -1
end_up = -1

with open('data/3250up3250downautomated.csv') as the_file:
    content = the_file.readlines()



numsamples = len(content)
time_vec = np.zeros(numsamples-1)
height_vec = np.zeros(numsamples-1)

for x in range(1,numsamples):


    valuestr = content[x].strip().split(',')

    if start_up < 0:
        if float(valuestr[5]) > 0:
            start_up = x-1
    elif end_up < 0:
        if float(valuestr[5]) <= .01:
            end_up = x-1

    time_vec[x-1] = float(valuestr[0])
    height_vec[x-1] = float(valuestr[1])

p = np.polyfit(time_vec[start_up:end_up-1],height_vec[start_up:end_up-1],2)


plt.title('Seq')
plt.xlabel('Time')
plt.ylabel('Height')
plt.plot(time_vec[start_up:end_up-1],height_vec[start_up:end_up-1])
plt.grid()
plt.show()