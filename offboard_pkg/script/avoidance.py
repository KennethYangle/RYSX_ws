import numpy as np
import time


class Avoidance:
    def __init__(self, vm):
        self.rt1 = 0.5
        self.rt2 = 1.0
        self.vm = vm
        self.k2 = 1.0
        self.em = 1e-6
        self.es = 1e-6

    def controller(self, rpos_body):
        ksimil = rpos_body[0]
        den = (1+self.em)*ksimil - self.rt1*self.mys(ksimil/self.rt1, self.es)
        bil = self.k2*self.dmysigma(ksimil, self.rt1, self.rt2)/den - self.k2*self.mysigma(ksimil, self.rt1, self.rt2)*((1+self.em)-self.dmys(ksimil/self.rt1, self.es)) / den**2
        V2 = bil*(np.array([1,0]))
        V = self.mysat(V2, self.vm)
        return V

    def mysat(self, v, vm):
        normv = np.linalg.norm(v)
        if normv > vm:
            return vm / normv * v
        else:
            return v

    def mys(self, x, e):
        x2 = 1 + e / np.tan( 67.5/180*np.pi )
        x1 = x2 - np.sin( 45/180*np.pi ) * e
        if x <= x1:
            return x
        elif x <= x2:
            return 1 - e + np.sqrt( e**2 - (x-x2)**2 )
        else:
            return 1.0

    def dmys(self, x, e):
        x2 = 1 + e / np.tan( 67.5/180*np.pi )
        x1 = x2 - np.sin( 45/180*np.pi ) * e
        if x <= x1:
            return 1.0
        elif x <= x2:
            return (x2 - x) / np.sqrt( e**2 - (x-x2)**2 )
        else:
            return 0.0

    def mysigma(self, x, d1, d2):
        if x <= d1:
            return 1.0
        elif x <= d2:
            A = -2 / (d1 - d2) ** 3
            B = 3 * (d1 + d2) / (d1 - d2) ** 3
            C = -6 * d1 * d2 / (d1 - d2) ** 3
            D = d2**2 * (3*d1 - d2) / (d1 - d2) ** 3
            return A * x**3 + B * x**2 + C * x + D
        else:
            return 0.0

    def dmysigma(self, x, d1, d2):
        if x <= d1:
            return 0.0
        elif x <= d2:
            A = -2 / (d1 - d2) ** 3
            B = 3 * (d1 + d2) / (d1 - d2) ** 3
            C = -6 * d1 * d2 / (d1 - d2) ** 3
            return 3*A * x**2 + 2*B * x + C
        else:
            return 0.0


if __name__ == "__main__":
    avo = Avoidance(3)
    pos_info = np.array([10, 10])
    print(avo.controller(pos_info))