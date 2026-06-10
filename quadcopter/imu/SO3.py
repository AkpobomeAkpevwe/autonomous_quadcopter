import numpy as np

class SO3:
    @classmethod
    def get_skew(cls, w):
        skew = np.array([[0, -w[2], w[1]],
                         [w[2], 0, -w[0]],
                         [-w[1], w[0], 0]])
        return skew

    @classmethod
    def get_dR(cls, omega, dt):
        w = omega * dt
        skew = cls.get_skew(w)
        th = np.sqrt(np.dot(w, w))
        if th == 0:
            dR = np.eye(3) 
        else:
            dR = np.eye(3)
            dR += np.sin(th) / th * skew 
            dR += (1 - np.cos(th)) / th ** 2 * skew@skew
        return dR
    
    @classmethod
    def get_nextR(cls, R, omega, dt):
        dR = cls.get_dR(omega, dt)
        R_next = R @ dR
        return R_next

