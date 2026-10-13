#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from math import pi
import math

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

L1 = 0.152
L2 = 0.120
L3 = 0.244
L4 = 0.093
L5 = 0.213
L6 = 0.104
L7 = 0.085
L8 = 0.092
L9 = 0.0535
L10 = 0.058

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6), dtype=float)

	# Home Configuration
	M = np.array([
		[0, -1, 0, 0.392],
		[0, 0, -1, 0.432],
		[1, 0, 0, 0.2155],
		[0, 0, 0, 1]
	], dtype=float)

	# Screw Twist
	q = np.zeros((6, 3), dtype=float)
	w = np.zeros((6, 3), dtype=float)
	v = np.zeros((6, 3), dtype=float)

	q[0] = np.array([-0.150, 0.150, 0.010])
	q[1] = np.array([q[0, 0], q[0, 1] + L2, q[0, 2] + L1])
	q[2] = np.array([q[1, 0] + L3, q[1, 1], q[1, 2]])
	q[3] = np.array([q[2, 0] + L5, q[2, 1] - L4, q[2, 2]])
	q[4] = np.array([q[3, 0], q[3, 1] + L6, q[3, 2]])
	q[5] = np.array([q[4, 0] + L7, q[4, 1] + L8, q[4, 2]])

	w[0] = np.array([0,0,1], dtype=float)
	w[1] = np.array([0,1,0], dtype=float)
	w[2] = np.array([0,1,0], dtype=float)
	w[3] = np.array([0,1,0], dtype=float)
	w[4] = np.array([1,0,0], dtype=float)
	w[5] = np.array([0,1,0], dtype=float)

	for i in range(6):
		v[i] = -np.cross(w[i], q[i])

	for i in range(6):
		S[0:3, i] = w[i]
		S[3:6, i] = v[i]

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	M, S = Get_MS()
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])

	T = M
	for i in range(5, -1, -1):
		twist = S[0:6, i]
		Si = twistToMat(s_twist=twist)

		T = expm(Si * theta[i]) @ T

	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + pi
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*pi)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value

def twistToMat(s_twist):
	w, v = s_twist[:3], s_twist[3:]
	mat = np.zeros((4, 4), dtype=float)
	mat[:3, :3] = skew(w=w)
	mat[:3, 3] = v
	return mat

def skew(w):
	return np.array([
		[0, -w[2], w[1]],
		[w[2], 0, -w[0]],
		[-w[1], w[0], 0]
	])


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#

	x_grip = xWgrip + 0.150
	y_grip = yWgrip - 0.150
	z_grip = zWgrip - 0.010
	yaw_rad = np.radians(yaw_WgripDegree)
	
	theta1 = 0.0
	theta2 = 0.0
	theta3 = 0.0
	theta4 = 0.0
	theta5 = 0.0
	theta6 = 0.0

	x_cen = x_grip - L9 * np.cos(yaw_rad)
	y_cen = y_grip - L9 * np.cos(yaw_rad)
	z_cen = z_grip

	theta1 = np.arctan(x_cen/y_cen)- np.arcsin((L2-L4+L6)/np.sqrt(x_cen**2 + y_cen**2))
	theta6 = np.pi / 2 + theta1 - yaw_rad


	d = L6 + 27

	x_3end = x_cen - L7*np.cos(theta1) + d*np.sin(theta1)
	y_3end = y_cen - L7*np.sin(theta1) - d*np.cos(theta1)
	z_3end = z_cen + L10 + L8

	_3end = np.array([x_3end, y_3end, z_3end])
	temp_length = np.linalg.norm(_3end- np.array([0,0,L1]))
	angle1 = (temp_length**2 + L3**2 - L5**2)/(2*temp_length*L3)
	angle2 = (L5**2 + L3**2 - temp_length**2)/(2*L5*L3)
	angle3 = (L5**2 + temp_length**2 - L3**2)/(2*L5*temp_length)

	theta2 = angle1 + np.sin(((z_3end-L1)/np.linalg.norm(_3end - np.array([0,0,L1]))))
	theta3 = np.pi - angle2
	theta4 = theta2 - theta3
	theta5 = -np.pi/2

	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
