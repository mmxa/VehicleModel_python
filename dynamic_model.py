#   reference: https://blog.csdn.net/AdamShan/article/details/78696874
#               Rajamani,Vehicle Dynamics and Control (Chinese edition)
#   author: MengXiangzhe
#   4,July,2019

import numpy as np
import matplotlib.pyplot as plt
import math

class Dynamic_model:
    def __init__(self, x, y, psi, v_x, f_len, r_len, k_f, k_r, mass, I_z):
        self.x = x
        self.y = y
        self.psi = psi
        self.v_x = v_x
        self.f_len = f_len
        self.r_len = r_len
        self.k_f = k_f
        self.k_r = k_r
        self.mass = mass
        self.I_z = I_z
        self.d_y = 0
        self.d_psi = 0

    def update_model(self, delta, dt):
        a_y = -2*(self.k_f + self.k_r)/self.mass/self.v_x * self.d_y - \
        (self.v_x + (2*(self.k_f * self.f_len - self.k_r * self.r_len)  \
        / self.mass / self.v_x) )* self.d_psi + 2 * self.k_f * delta / self.mass
        a_psi = (-2 * (self.f_len*self.k_f - self.r_len*self.k_r) / self.I_z / \
        self.v_x ) * self.d_y - 2 * (self.k_f * self.f_len ** 2 + 2 * \
        self.k_r * self.r_len ** 2) / self.I_z / self.v_x * self.d_psi \
        + 2 * self.f_len * self.k_f * delta / self.I_z
        d_y = self.d_y                                                          #recording the state of previous step
        d_psi = self.d_psi
        self.d_psi = self.d_psi + a_psi * dt
        self.d_y = self.d_y + a_y * dt
        self.psi = self.psi + d_psi * dt                                        #update state for vehicle in global coordinates
        dx = self.v_x * math.cos(self.psi) - d_y * math.sin(self.psi)           #transform to global coordinates
        dy = self.v_x * math.sin(self.psi) + d_y * math.cos(self.psi)
        self.x = self.x + dx * dt
        self.y = self.y + dy * dt


car = Dynamic_model(x=0, y=0, psi=0, v_x=5, f_len=1.4, r_len=1.5, k_f=62618, k_r=140118, mass=1800, I_z=3889)
his_x = []
his_y = []
step = 0
delta = 0
#plt.axis([-10, 50, -10, 50])
while step < 1000:
    car.update_model(delta=0.2, dt=0.01)
    his_x.append(car.x)
    his_y.append(car.y)
    step = step + 1
    #print("now it runs at: ", step, "steps!")                                  #for testing code
plt.plot(his_x, his_y, "-k")
plt.pause(0.01)
plt.show()
