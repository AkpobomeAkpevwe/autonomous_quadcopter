import numpy as np

from imu.SO3 import SO3
from imu.RotationUtils import RotationUtils
from imu.SensorUtils import SensorUtils


class DCM:
    def __init__(self):
        self.dcm_b2g = np.eye(3)
        self.prev_time = 0
        self.kp = 0.15
        self.euler = np.zeros(3)
    
    def calculate_dt(self, curr_time):
        if (self.prev_time == 0):
            self.prev_time = curr_time
            return -1
        dt = curr_time - self.prev_time
        self.prev_time = curr_time
        return dt

    def orthonormalization(self, R):
        error = np.dot(R[:,0], R[:,1])
        # orthogonalization
        R0 = R[:,0] - error/2*R[:,1]
        R1 = R[:,1] - error/2*R[:,0]
        R2 = np.cross(R0, R1)
        # renormalization
        R[:,0] = 0.5*(3 - np.dot(R0, R0))*R0
        R[:,1] = 0.5*(3 - np.dot(R1, R1))*R1
        R[:,2] = 0.5*(3 - np.dot(R2, R2))*R2
        return R

    def err_meas_acc(self,R, acc):
        acc_scaled = acc/9.81
        g_true = [0, 0, -1]
        acc_gnd = R @ acc_scaled
        err_gnd_acc = np.cross(acc_gnd, g_true)
        return err_gnd_acc

    def err_meas_mag(self, R, mag):
        mag_scaled = mag/np.linalg.norm(mag)
        mag_true = [1, 0, 0]
        mag_gnd = R @ mag_scaled
        err_gnd_mag = np.cross(mag_gnd, mag_true)
        return err_gnd_mag

    def err_meas_bdy(self, R, acc, mag):
        err_meas_gnd = self.err_meas_acc(R, acc) + self.err_meas_mag(R, mag)
        err_meas_bdy = np.matmul(R.T, err_meas_gnd)        
        return err_meas_bdy
        
    def update(self, gyro_si, acc_si, mag_si, curr_time):
        dt = self.calculate_dt(curr_time)
        if dt < 0:
            return False
        omega = gyro_si
        
        # calcualte the next DCM
        self.dcm_b2g = SO3.get_nextR(self.dcm_b2g, omega, dt)
        self.dcm_b2g = self.orthonormalization(self.dcm_b2g)
        
        # calculate the error
        err_meas_bdy = self.err_meas_bdy(self.dcm_b2g, acc_si, mag_si)

        # correction
        Rerr = SO3.get_dR(err_meas_bdy, self.kp)
        self.dcm_b2g  = self.dcm_b2g @ Rerr

        # euler
        self.euler = RotationUtils.rotm2eul(self.dcm_b2g)

        return True

    
