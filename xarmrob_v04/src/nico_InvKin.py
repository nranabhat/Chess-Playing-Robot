import scipy.optimize as optimize
import numpy as np
from FwdKinArmRob_serial import armrobfwdkin

def endpoint_error(angles, args):
    true_endpoint = args
    x_true = true_endpoint[0]
    y_true = true_endpoint[1]
    alpha = np.arctan2(y_true, x_true)
    beta1 = angles[0]
    beta2 = angles[1]
    beta3 = angles[2]
    gamma3 = 0.0
    gamma5 = 0.0
    all_angles = [alpha, beta1, beta2, gamma3, beta3, gamma5]

    estimated_endpoint = armrobfwdkin(all_angles)  # in radians 
    diff = true_endpoint - estimated_endpoint
    error = np.linalg.norm(diff)
    return error

def calculate_angles(endpoint, angle_guess):
    bounds = [(-3*np.pi/4, 0),  # Bounds for beta1
            (0, np.pi),  # Bounds for beta2
            (0, np.pi/2)]  # Bounds for beta3. have gripper above the pieces

    # Perform the optimization with bounds
    calculated_beta_angles = optimize.minimize(endpoint_error, angle_guess, args=(endpoint), method="SLSQP", bounds=bounds)

    return calculated_beta_angles

def nico_IK(endpoint):
    """ Params: enpoint: list [x,y,z]
                angle_guesses: list [x0,y0,z0] 
                
        Retern: angles: float array of [alpha,b1,b2,g1,b3,g2]"""
    angle_guess = [-np.pi/4, np.pi/2, np.pi/4]
    beta_angles = calculate_angles(endpoint, angle_guess).x
    alpha = np.arctan2(endpoint[1], endpoint[0])
    angles = [alpha, beta_angles[0], beta_angles[1], 0, beta_angles[2], 0]
   
    return np.asfarray(angles)