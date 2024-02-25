import numpy as np

class Drone3D:
	def __init__(self, dt=0.01,
					   kf=1.5e-6,
					   km=1.5e-7,
					   ix=0.01,
					   iy=0.01,
					   iz=0.01,
					   l=0.5):
		self.dt = dt
		self.kf = kf
		self.km = km
		self.ix = ix
		self.iy = iy
		self.iz = iz
		self.l = l
		self.omega_1 = 0.
		self.omega_2 = 0.
		self.omega_3 = 0.
		self.omega_4 = 0.
		

		self.x = 0
		self.y = 0
		self.z = 0
		self.xdot = 0
		self.ydot = 0
		self.zdot = 0


		self.phi = 0.
		self.theta = 0.
		self.psi = 0.
		self.p = 0
		self.q = 0
		self.r = 0

		self.m = 0.25
		self.g = 9.81

	@property
	def f(self):
		return self.f1 + self.f2 + self.f3 + self.f4

	@property
	def f1(self):
		return self.kf * self.omega_1**2

	@property
	def f2(self):
		return self.kf * self.omega_2**2

	@property
	def f3(self):
		return self.kf * self.omega_3**2

	@property
	def f4(self):
		return self.kf * self.omega_4**2

	@property
	def m1(self):
		return -self.km * self.omega_1**2

	@property
	def m2(self):
		return self.km * self.omega_2**2

	@property
	def m3(self):
		return self.km * self.omega_3**2

	@property
	def m4(self):
		return -self.km * self.omega_4**2 

	@property
	def mx(self):
		return self.l * (self.f3 + self.f4 - self.f1 - self.f2)

	@property
	def my(self):
		return self.l * (self.f1 + self.f4 - self.f2 - self.f3)

	@property
	def mz(self):
		return self.m1 + self.m2 + self.m3 + self.m4

	def R(self):
		phi = self.phi
		theta = self.theta
		psi = self.psi

		rx = np.array([[1, 0, 0], [0, np.cos(phi), -np.sin(phi)], [0, np.sin(phi), np.cos(phi)]])
		ry = np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])
		rz = np.array([[np.cos(psi), -np.sin(psi), 0],[np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
		return rz @ ry @ rx

	def advance_state(self, omega_1, omega_2, omega_3, omega_4):
		self.omega_1 = omega_1
		self.omega_2 = omega_2
		self.omega_3 = omega_3
		self.omega_4 = omega_4
		self.advance_attitude()
		self.advance_position()

	def advance_attitude(self):
		p_dot = self.mx / self.ix
		q_dot = self.my / self.iy
		r_dot = self.mz / self.iz

		self.p += p_dot * self.dt
		self.q += q_dot * self.dt
		self.r += r_dot * self.dt

		transformation_matrix = np.array([  [1, np.sin(self.phi)*np.tan(self.theta), np.cos(self.phi)*np.tan(self.theta)],
											[0, np.cos(self.phi), -np.sin(self.phi)],
											[0, np.sin(self.phi)/np.cos(self.theta), np.cos(self.phi)/np.cos(self.theta)]])

		euler_rates = transformation_matrix @ np.array([self.p, self.q, self.r])
		phi_dot, theta_dot, psi_dot = euler_rates

		self.phi += phi_dot * self.dt
		self.theta += theta_dot * self.dt
		self.psi += psi_dot * self.dt


	def advance_position(self):

		xddot, yddot, zddot = (self.R() @ np.array([0,0,self.f - self.m * self.g])) / self.m
		self.xdot += xddot * self.dt
		self.ydot += yddot * self.dt
		self.zdot += zddot * self.dt

		self.x += self.xdot * self.dt
		self.y += self.ydot * self.dt
		self.z += self.zdot * self.dt 



		

# Create a class that is capable of updating the state of a 3D drone, given a current state, action input (in the form of four motor thrusts), and an integration timestep.


