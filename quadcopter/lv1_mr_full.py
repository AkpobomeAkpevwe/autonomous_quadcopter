import numpy as np
import matplotlib.pyplot as plt
from ubicoders_vrobots import VRobotGraphClient, VRDataWriter, VRobotState, VirtualRobot, vr_client_main
from mr_modules.controllers.AttController import AttController
from mr_modules.controllers.LinVelController import LinVelController
from mr_modules.controllers.LinPosController import LinPosController
from mr_modules.controllers.RateController import RateController
np.set_printoptions(suppress=True, precision=2)

realtime_plot = VRobotGraphClient()



def vec3_to_np(vec3):
    """Convert a Vec3 object to a numpy array 1x3"""
    return np.array([vec3.x, vec3.y, vec3.z])


class MyVRobotsController:
    def __init__(self, vr: VirtualRobot = None):
        self.vr = vr

        self.wp = np.array([[50.6, 180, -30],
                            [60, 100, -20]])
       
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
        if wp_error_norm < 1:
            if (self.wp_idx + 1) == len(self.wp):
                pass
            else:
                self.wp_idx += 1
        #=========================================
        
        # ========================================
        # computing linear position error
        linPos_gt = vec3_to_np(states.linPos)
        pos_error = self.pos_des - linPos_gt
        self.vel_des = self.linPosCtrl.update(pos_error)       

        # realtime_plot.update(ts, linPos_gt)
        # ========================================

        # ========================================
        # Velcity controller
        linVel_gt = vec3_to_np(states.linVel)
        vel_error = self.vel_des - linVel_gt
        cmd_vel = self.linVelCtrl.update(vel_error)
        self.att_des = np.array([cmd_vel[1], -cmd_vel[0], 0])
        # print("cmd_vel: ", cmd_vel)
        # realtime_plot.update(ts, linVel_gt)
        # ========================================

        # ========================================
        # Attittude Controller
        euler_gt = vec3_to_np(states.euler) * np.pi/180
        att_error = self.att_des - euler_gt
        self.rate_des = self.att_ctrl.update(att_error)
        # realtime_plot.update(ts, euler_gt)
        # ========================================
        

        # ========================================
        # Rate Controller
        angVel_gt = vec3_to_np(states.angVel)
        rate_error = self.rate_des - angVel_gt
        cmd_rate = self.rate_ctrl.update(rate_error)
        # print("cmd_rate: ", cmd_rate)
        # realtime_plot.update(ts, angVel_gt)       
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
    vr_client_main(controller, duration=60)  # Interact for 60 seconds    
    
    print("simulation finished")
