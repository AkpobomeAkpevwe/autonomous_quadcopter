from scipy import integrate
import  numpy as np

class InvPenEnvironment():
    def __init__(self, ic=[0.0, 0.0, np.pi, 0.01], m=5, M=10, L=1, g=-9.8, ):
        self.x = ic
        self.m = m
        self.M = M
        self.L = L
        self.g = g
        self.d = 1  # friction(damping) term
        self.u = 0  # ctrl input
        self.r = integrate.ode(self.get_dx).set_integrator("dopri5")  # dori5= runge kutta 45

    def get_next(self, u, dt):
        t = np.array([0, dt])
        # intital state = current state & initial time = 0
        self.r.set_initial_value(self.x, t[0])
        self.u = u
        self.x = self.r.integrate(dt)
        # self.x = self.x + self.get_dx(0, self.x)*self.dt
        return self.x

    def get_dx(self, t, x):
        sinx = np.sin(x[2])
        cosx = np.cos(x[2])
        denom = self.m*self.L**2*(self.M+self.m*(1-cosx**2))
        dx = ([0, 0, 0, 0])
        dx[0] = x[1]
        dx[1] = (1/denom)*(-self.m**2*self.L**2*self.g*cosx*sinx + self.m*self.L**2 *
                           (self.m*self.L*x[3]**2*sinx - self.d*x[1])) + self.m*self.L**2*(1/denom)*self.u
        dx[2] = x[3]
        dx[3] = (1/denom)*((self.m+self.M)*self.m*self.g*self.L*sinx - self.m*self.L*cosx *
                           (self.m*self.L*x[3]**2*sinx - self.d*x[1])) - self.m*self.L*cosx*(1/denom)*self.u
        return dx