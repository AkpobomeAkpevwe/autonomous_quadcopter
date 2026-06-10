import numpy as np

class RotationUtils:
    @classmethod
    def eul2rotm(cls, euler):
        x, y, z = euler
        Rx = np.array([[1, 0, 0],
                        [0, np.cos(x), -np.sin(x)],
                        [0, np.sin(x), np.cos(x)]],)
    
        Ry = np.array([[np.cos(y), 0,  np.sin(y)],
                        [0, 1,  0],
                        [-np.sin(y), 0, np.cos(y)]])
        
        Rz = np.array([[np.cos(z), -np.sin(z),  0],
                        [np.sin(z), np.cos(z), 0],
                        [0, 0, 1]])
        
        return Rz@Ry@Rx

    @classmethod
    def rotm2eul(cls, R):
        sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
        phi = np.arctan2(R[2,1], R[2,2])
        theta = np.arctan2(-R[2,0], sy)
        psi = np.arctan2(R[1,0], R[0,0])
        return np.array([phi, theta, psi])



