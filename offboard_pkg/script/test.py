import numpy as np
from Queue import Queue

a = np.random.randint(0, 100, 300)
print(a)
q = Queue()
maxQ = 100
sumQ = 0.0
b = []

for yaw in a:
    if q.qsize() < maxQ:
        sumQ += yaw
        q.put(yaw)
        car_yaw = sumQ / q.qsize()
    else:
        sumQ += yaw
        q.put(yaw)
        first_yaw = q.get()
        sumQ -= first_yaw
        car_yaw = sumQ / maxQ
    b.append(car_yaw)
print(b)