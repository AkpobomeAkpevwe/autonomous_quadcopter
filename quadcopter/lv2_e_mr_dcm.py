import numpy as np
import matplotlib.pyplot as plt
from ubicoders_vrobots import VRobotGraphClient, VRDataWriter, VRobotState, VirtualRobot, vr_client_main
from mr_modules.controllers.AttController import AttController
from mr_modules.controllers.LinVelController import LinVelController
from mr_modules.controllers.LinPosController import LinPosController
from mr_modules.controllers.RateController import RateController
from imu.DCM import DCM
from imu.SensorUtils import SensorUtils
np.set_printoptions(suppress=True, precision=2)

realtime_plot = VRobotGraphClient()


def vec3_to_np(vec3):
    """Convert a Vec3 object to a numpy array 1x3"""
    return np.array([vec3.x, vec3.y, vec3.z])


class MyVRobotsController:
    def __init__(self, vr: VirtualRobot = None):
        self.vr = vr

        self.dcm = DCM()
        self.omega_lp = np.zeros(3)

        self.wp = np.array([[50, 180, -30, 0],
                            [50, 150, -30, -np.pi/2],
                            [70, 150, -30, 0],
                            [70, 180, -30, np.pi/2],
                            [50, 180, -30, 0]
                            ])
        self.wp_idx = 0

        # linear position controller
        # self.pos_des = np.array([60, 100, -20])
        self.linPosCtrl = LinPosController()

        # linear velocity controller
        #self.vel_des = np.array([2.321, -1.123, -3.5])
        self.linVelCtrl = LinVelController()

        # attitude controller
        self.att_ctrl = AttController()
        #self.att_des = np.array([-5, -5, -5])*np.pi/180

        # rate controller
        self.rate_ctrl = RateController()
        #self.rate_des = np.array([0, 0, 10])*np.pi/180

    def loop(self):
        # Retrieve the states
        states: VRobotState = self.vr.states

        # Reading the data - time stamp           
        ts = states.timestamp/1000 # seconds.

        #=========================================
        # Waypoint following
        self.pos_des = self.wp[self.wp_idx][:3]
        wp_error = self.pos_des - vec3_to_np(states.linPos)
        wp_error_norm = np.linalg.norm(wp_error)
        if wp_error_norm < 10:
            if (self.wp_idx + 1) == len(self.wp):
                pass
            else:
                self.wp_idx += 1
        #=========================================

        #=========================================
        # State Estimation
        linPos_gt = vec3_to_np(states.linPos)
        linVel_gt = vec3_to_np(states.linVel)

        gyro_si = SensorUtils.cvt_gyro_si(vec3_to_np(states.gyroscope))
        acc_si = SensorUtils.cvt_acc_si(vec3_to_np(states.accelerometer))
        mag_si = SensorUtils.cvt_mag_si(vec3_to_np(states.magnetometer))

        R_b2g = self.dcm.dcm_b2g
        linacc_gt = vec3_to_np(states.linAcc)
        lin_acc_body = R_b2g @ linacc_gt
        gravity_acc_body = acc_si - lin_acc_body
        self.dcm.update(gyro_si, gravity_acc_body, mag_si, ts)

        euler_est = self.dcm.euler
        angVel_est = gyro_si
        #=========================================

        
        # ========================================
        # computing linear position error
        pos_error = self.pos_des - linPos_gt
        self.vel_des_gnd = self.linPosCtrl.update(pos_error)
        
        self.vel_des = R_b2g.T @ self.vel_des_gnd
        # ========================================

        # ========================================
        # Velcity controller
        vel_error = self.vel_des - linVel_gt
        cmd_vel = self.linVelCtrl.update(vel_error)
        heading_des = self.wp[self.wp_idx][3]
        self.att_des = np.array([cmd_vel[1], -cmd_vel[0], heading_des])
        # ========================================

        # ========================================
        # Attittude Controller        
        att_error = self.att_des - euler_est
        self.rate_des = self.att_ctrl.update(att_error)
        # ========================================
        

        # ========================================
        # Rate Controller  
        rate_error = self.rate_des - angVel_est        
        cmd_rate = self.rate_ctrl.update(rate_error)
        # ========================================
        
        # ========================================
        # MIXER
        throttle = 1400 - cmd_vel[2]
        # Update Actuators
        throttle = np.clip(throttle, 1000, 2000)    

        m0 = throttle + cmd_rate[0] + cmd_rate[1] - cmd_rate[2]
        m1 = throttle - cmd_rate[0] + cmd_rate[1] + cmd_rate[2]
        m2 = throttle - cmd_rate[0] - cmd_rate[1] - cmd_rate[2]
        m3 = throttle + cmd_rate[0] - cmd_rate[1] + cmd_rate[2]
        self.vr.update_cmd_multirotor(0, [m0, m1, m2, m3])
        # ========================================


# Main code
if __name__ == "__main__":
    # Instantiate the controller
    controller = MyVRobotsController()

    # Run the controller without modifying your main loop
    vr_client_main(controller, duration=90)  # Interact for 60 seconds    
    
    print("simulation finished")
