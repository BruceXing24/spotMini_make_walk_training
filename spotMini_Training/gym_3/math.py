import plistlib

import numpy as np
import matplotlib.pyplot as plt
import  time

PI = np.pi
n = 0

t_list  = []
A_list  = []
Ao_list = []
A5_list = []
while n < 200:
    R = np.random.uniform(0,2,1)
    t = 0.01*n
    A = R* np.sin( 10 * PI/3 * t )
    A_5 = 5 * np.sin(10 * PI / 3 * t)
    A_o = np.sin( 10 * PI/3 * t )
    Ao_list.append(A_o)
    t_list.append(t)
    A_list.append(A)
    A5_list.append(A_5)
    n+=1


fig =  plt.figure
plt.plot(t_list, A_list)
plt.plot(t_list, Ao_list)
plt.plot(t_list, A5_list)
plt.show()





