import numpy as np

class SensorUtils:
    @classmethod
    def cvt_acc_si(cls, acc):
        return acc / 16384.0 * 9.81
    
    @classmethod
    def cvt_mag_si(cls, mag):
        return mag / 45.0

    @classmethod
    def cvt_gyro_si(cls, gyro):
        return gyro * 250 / 32768 * np.pi / 180  

    @classmethod
    def acc2eul(cls, acc):
        g = 9.81
        theta = np.arcsin(acc[0]/g)
        phi = -np.arcsin(acc[1]/(g*np.cos(theta)))
        return np.array([phi, theta, 0])
    
    @classmethod
    def mag2eul(cls, mag, euler_xy):
        phi = euler_xy[0]
        theta = euler_xy[1]
        mx, my, mz = mag/np.linalg.norm(mag)        
        sinphi = np.sin(phi)
        sinthe = np.sin(theta)
        cosphi = np.cos(phi)
        costhe = np.cos(theta)
        cospsi= mx / costhe
        sinpsi= (-my + sinphi*sinthe*cospsi)/cosphi
        psi = np.arctan2(sinpsi, cospsi)
        return np.array([phi, theta, psi])
