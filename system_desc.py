import scipy.io as spio
import os
from pathlib import Path
import numpy as np
import subprocess
from scipy.linalg import expm, solve_discrete_lyapunov
from scipy.signal import cont2discrete
import control as ctrl

# --------------------------
# if __name__ == "__main__":
def system_desc(system):
    # -------------------------- System Description -------------------------- #

    if system == 'suspension_control':
    # -------------------system ='suspension_control'-------------------------- #
    # system matrices (from Revisiting...emsoft 2024, can load from .mat file) 
    # Multi-Objective Co-Optimization of FlexRay-based Distributed Control System)--in CCM ECU
    # car position, velocity, suspension position, suspension velocity -- states
    # control input -- force applied to the suspension
    # y -- measured output (car position)

        a = np.array([[0, 1, 0, 0],
                        [-8, -4, 8, 4],
                        [0, 0, 0, 1],
                        [80, 40, -160, -60]])
        b = np.array([[0],
                        [80],
                        [20],
                        [-1120]])
        ad = np.array([[0.995860265603438, 0.0378696105301410, 0.00212669277812880, 0.00160492825553481],
                        [-0.174562623798343, 0.908578953704267, 0.0461683633555589, 0.0573094395666044],
                        [0.0311138708080227, 0.0160492825553481, 0.935759216765522, 0.0151544413769853],
                        [1.08396104971604, 0.573094395666044, -2.29631635987487, 0.0906898643677941]])
        bd = np.array([[0.0341116067291290],
                        [1.27458305177487],
                        [0.336215739966162],
                        [-16.9738474024853]])
        c = np.array([[1,0,0,0]])
        k = np.array([[3.698404852057683,0.527354601469353,-0.056946984711637,0.031194507232084]])  
        l = np.array([[0.627216669816836],
                        [0.077667216653486],
                        [0.033503186827617],
                        [0.804439585315088]])   
        h = 0.04
        # ad, bd, cd, dd, Ts = ctrl.c2d(a, b, c, np.array([[0]]), h)
        mdadt1 = 4
        dims = len(a)
        curdir = os.getcwd()
        matfilepath = os.path.join(curdir, f'{system}', f'{system}.mat')
        # # Load the .mat file
        # print(f"Loading .mat file from {matfilepath}")
        # # mat_contents = spio.loadmat(matfilepath)

        # try:
        #     # Load the .mat file
        #     print(f"Loading .mat file from {matfilepath}")
        #     mat_contents = spio.loadmat(str(matfilepath))
        #     print("Variables in .mat file:", mat_contents.keys())
        # except FileNotFoundError:
        #     print(f"Error: Could not find .mat file at {matfilepath}")
        #     print("Please check if the file exists and the path is correct")
        # except Exception as e:
        #     print(f"Error loading .mat file: {e}")

        safex = np.array([[-2, -20, -10, -60],
                            [ 2, 20, 10, 60]])

        # Grid granularity for partitioning X_S
        grid_delta = [2, 20, 10, 60] # [1, 100, 50, 600]

        # locations = [1, 10, 100, 1000]
        locations = 4  # number of digits in location names, e.g. loc=4 -> [1, 10, 100, 1000]

    return (system, a, b, c, ad, bd, k, l, h, safex, grid_delta, dims, mdadt1, locations)

if __name__ == "__main__":
    system_desc(system='suspension_control')