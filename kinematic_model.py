import math
import matplotlib.pyplot as plt
#reference: https://blog.csdn.net/AdamShan/article/details/78696874
#author: "MengXiangzhe"
#time: "3 July 2019"

class Kinematic_model:
    def __init__(self, x=0, y=0, psi=0, v=1, f_len=1.3, r_len=1.5):
        self.x = x
        self.y = y
        self.psi = psi
        self.v = v
        self.f_len = f_len
        self.r_len = r_len

    def get_state(self):
        return self.x, self.y, self.psi, self.v

    def update_state(self, a, delta, dt):
        beta = math.atan((self.r_len / (self.r_len + self.f_len)) * math.tan(delta))
        self.x = self.x + self.v * math.cos(self.psi + beta) * dt
        self.y = self.y + self.v * math.sin(self.psi + beta) * dt
        self.psi = self.psi + self.v / self.r_len * math.sin(beta) * dt
        self.v = self.v + a * dt

car = Kinematic_model(x=5, y=5, psi=0, v=5, f_len=1.3, r_len=1.5)
step = 1000                   #define a step for simulate
dt = 0.01
his_x = []                    #recording a track
his_y = []
plt.axis([-10, 50, -10, 50])
print ("the simulate time is :  ", dt*step)

while (step > 0):         #update model and restore the state
    his_x.append(car.x)
    his_y.append(car.y)
    car.update_state(0.02, 0.1+step*0.001, dt)
    step = step - 1
    plt.plot(his_x, his_y, "-k")
    plt.pause(0.01)
plt.show()
