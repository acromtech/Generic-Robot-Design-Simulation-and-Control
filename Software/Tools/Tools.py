"""
The Tools module provides a collection of utility methods for robotic simulations and analyses.

Dependencies:
    - NumPy: A fundamental package for scientific computing with Python.
        https://pypi.org/project/numpy/
        pip3 install numpy
    - Matplotlib: A plotting library for creating static, animated, and interactive visualizations in Python.
        https://pypi.org/project/matplotlib/
        pip3 install matplotlib

Please ensure that these dependencies are installed before using the functionalities provided by this module.

You can learn more about Khalil-Dombre_Modelisation : https://www.gdr-robotique.org/cours_de_robotique/online/Khalil-Dombre_Modelisation/Khalil-Dombre_Modelisation.pdf
You can check also the pinochio python library : https://github.com/stack-of-tasks/pinocchio/tree/master/examples
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Tools:
    
    # DHM PARAMETERS
    a = []      # List of a_{i-1} DHM parameter         -> dist(O_{i-1},O_i) suivant l'axe X_{i-1}  - ex : [a0,a1,a2,...] 
    alpha = []  # List of alpha_{i-1} DHM parameter     -> ang(Z_{i-1},Zi) selon l'axe X_{i-1}      - ex : [alpha0,alpha1,alpha2,...]
    r = []      # List of r_i DHM parameter             -> dist(O_{i-1},Oi) suivant l'axe Z_{i-1}   - ex : [r1,r2,r3,...]

    # TRAJECTORY PARAMETERS
    A = [0.,0.,0.]          # Starting position of the trajectory [xA, yA, zA] (list[float])
    B = [0.,0.,0.]          # Ending position of the trajectory [xB, yB, zB] (list[float])
    V1 = 0.                 # Initial velocity (float)
    V2 = 0.                 # Final velocity (float)
    t = 0                   # Time (float)
    T = 0                   # Total time (float)
    s = []                  # Trajectory (list[float])
    Xs = []                 # Operational trajectory (list[float])
    x = []                  # Evolution of x coordinate (list[float])
    y = []                  # Evolution of y coordinate (list[float])
    z = []                  # Evolution of z coordinate (list[float])
    endEffector_speed = []  # End effector speed profile (list[float])


    def __init__(self, joint_types=[], joint_distances=[], joint_orientations=[], q_config=[]):
        """
        Initialize the Tools class.

        Parameters:
            joint_types : list[int], optional
                List of joint types (0=Prismatic, 1=Revolute). Defaults to an empty list.
            joint_distances : list[float], NOT optional
                List of distances between each joints (in mm). Defaults to an empty list.
            joint_orientations : list[int], optional
                List of orientations of each joints (0=X, 1=Y, 2=Z). Defaults to an empty list.
            q_config : list[float], NOT optional
                List of configurations of q. Defaults to an empty list.

        Initializes the joint types, joint distances, joint orientations, and q configurations 
        based on the provided parameters. Additionally, calculates the DHM parameters based on 
        the initialized joint types, joint distances, and joint orientations.
        """
        # Initialize DHM parameters with default values
        num_joints = len(q_config)
        self.a = [0] * num_joints
        self.alpha = [0] * num_joints
        self.r = [0] * num_joints

        self.joint_types = joint_types
        self.joint_distances = joint_distances
        self.joint_orientations = joint_orientations
        self.q_config = q_config
        if joint_types is not None : self.calculate_DHM_parameters()


    # DHM PARAMETERS ================================================================================================================
    def calculate_DHM_parameters(self):
        """
        Calculate DHM parameters for each joints of the robot features

        Parameters :
            joint_types          :list[int]              # List of joint type (0=Prismatic, 1=Revolute)                      - ex : Robot RPPR -> [1,0,0,1]
            joint_distances      :list[float]            # List of contants - distance beetween each joints (in mm)          - ex : [o0o1,o1o2,o2o3,...]
            joint_orientations   :list[int]              # List of contants - orientation of each joints (0=X, 1=Y, 2=Z)     - ex : [0,0,1,...]

        Return :
            a                   :list[float]            # List of a_{i-1} DHM parameter         -> dist(O_{i-1},O_i) suivant l'axe X_{i-1}  - ex : [a0,a1,a2,...] 
            alpha               :list[float]            # List of alpha_{i-1} DHM parameter     -> ang(Z_{i-1},Zi) selon l'axe X_{i-1}      - ex : [alpha0,alpha1,alpha2,...]
            r                   :list[float]            # List of r_i DHM parameter             -> dist(O_{i-1},Oi) suivant l'axe Z_{i-1}   - ex : [r1,r2,r3,...]
        """
        # CODE AN ALGORITH TO SET DHM PARAMETERS AUTOMATICLY FOR GENERIC USE-CASES
        # self.a = 
        # self.alpha = 
        # self.r =
    
    def set_DHM_parameters(self, a=None, alpha=None, r=None):
        """
        Set DHM parameters for each joint of the robot features.
        Each parameter is optional

        Parameters :
            a : list[float] or None, optional
                List of a_{i-1} DHM parameters.
            alpha : list[float] or None, optional
                List of alpha_{i-1} DHM parameters.
            r : list[float] or None, optional
                List of r_i DHM parameters.

        Raises:
            ValueError: If the length of the provided parameter lists does not match the current number of joints.

        Example:
            - `set_DHM_parameters(a=[10, 20, 30], alpha=[0.1, 0.2, 0.3], r=[100, 200, 300])`
            - `set_DHM_parameters(a=[,,30], alpha=[0.1,, 0.3], r=[, 200, 300])`
        """
        if a is not None:
            if len(a) != len(self.joint_distances):
                raise ValueError("Length of parameter 'a' does not match the current number of joints")
            for i in range(len(a)):
                if a[i] is not None:
                    self.a[i] = a[i]

        if alpha is not None:
            if len(alpha) != len(self.joint_distances):
                raise ValueError("Length of parameter 'alpha' does not match the current number of joints")
            for i in range(len(alpha)):
                if alpha[i] is not None:
                    self.alpha[i] = alpha[i]

        if r is not None:
            if len(r) != len(self.joint_distances):
                raise ValueError("Length of parameter 'r' does not match the current number of joints")
            for i in range(len(r)):
                if r[i] is not None:
                    self.r[i] = r[i]

    def get_DHM_parameters(self, parameter=None):
        """
        Get DHM parameters for each joint of the robot features.

        Parameters :
            parameter : str or None, optional
                If specified, returns the value of the given parameter.
                If None, returns all DHM parameters.

        Returns :
            If parameter is specified:
                The value of the specified parameter.
            If parameter is None:
                A dictionary containing all DHM parameters:
                {
                    'a': self.a,
                    'alpha': self.alpha,
                    'r': self.r
                }
        """
        parameters = {
            'a': self.a,
            'alpha': self.alpha,
            'r': self.r
        }

        if parameter is not None:
            if parameter not in parameters:
                raise ValueError("Invalid parameter name")
            return parameters[parameter]
        else:
            return parameters

    # JOINT PARAMETERS ================================================================================================================
    def set_joint_parameters(self, joint_types=None, joint_distances=None, joint_orientations=None, q_config=None):
        """
        Set joint parameters of the robot 
        Each parameter is optional

        Parameters :
            joint_types         : List of joint types (0=Prismatic, 1=Revolute) - ex : [1,0,0,1,...]
            joint_distances     : List of contants - distance between each joints (in mm) - ex : [o0o1,o1o2,...]
            joint_orientations  : List of contants - orientation of each joints (0=X, 1=Y, 2=Z) - ex : [0,0,1,...]
            q_config            : List of constants - configuration of q - ex : [q1,q2,q3,...]

        Examples:
        `set_joint_parameters(joint_types=[0,,1], joint_distances=[,,80], q_config=[,0,])`
        - Sets the joint types for the first joint to Revolute (1) and third joints to Prismatic (0), 
        - Sets the distance between the third and fourth joints to 80mm,
        - Does not modify the joint orientations,
        - Sets the value of q for the second joint to 0.
        """
        if joint_types is not None:
            for i in range(len(joint_types)):
                if joint_types[i] is not None:
                    self.joint_types[i] = joint_types[i]

        if joint_distances is not None:
            for i in range(len(joint_distances)):
                if joint_distances[i] is not None:
                    self.joint_distances[i] = joint_distances[i]

        if joint_orientations is not None:
            for i in range(len(joint_orientations)):
                if joint_orientations[i] is not None:
                    self.joint_orientations[i] = joint_orientations[i]

        if q_config is not None:
            for i in range(len(q_config)):
                if q_config[i] is not None:
                    self.q_config[i] = q_config[i]

    def get_joint_parameters(self, parameters=None):
        """
        Get joint parameters of the robot.

        Parameters:
            parameters : str or list of str or None, optional
                If specified, returns the value of the given parameter or parameters.
                If None, returns all joint parameters.
                If a single parameter name or a list of parameter names is provided, returns the corresponding values.

        Returns:
            If parameter is specified:
                The value of the specified parameter.
            If parameter is None:
                A dictionary containing all joint parameters.
        """
        all_parameters = {
            'joint_types': self.joint_types,
            'joint_distances': self.joint_distances,
            'joint_orientations': self.joint_orientations,
            'q_config': self.q_config
        }

        if parameters is None:
            return all_parameters

        if isinstance(parameters, str):
            parameters = [parameters]

        requested_parameters = {}
        for param in parameters:
            if param in all_parameters:
                requested_parameters[param] = all_parameters[param]
            else:
                raise ValueError(f"Invalid parameter name: {param}")

        # If only one parameter is requested, return its value directly
        if len(requested_parameters) == 1:
            return list(requested_parameters.values())[0]

        return requested_parameters


    # TRAJECTORY PARAMETERS ================================================================================================================
    def set_trajectory_parameters(self, A=None, B=None, V1=None, V2=None):
        """
        Set trajectory parameters for the robot.

        Parameters:
            A : list[float] or None, optional
                Starting position of the trajectory [xA, yA, zA].
            B : list[float] or None, optional
                Ending position of the trajectory [xB, yB, zB].
            V1 : float or None, optional
                Initial velocity.
            V2 : float or None, optional
                Final velocity.

        Example:
            set_trajectory_parameters(A=[1.0, 2.0, 3.0], B=[4.0, 5.0, 6.0], V1=10.0, V2=5.0)
        """
        if A is not None:
            if len(A) != 3:
                raise ValueError("Parameter 'A' must be a list of three floats [xA, yA, zA]")
            for i in range(len(A)):
                if A[i] is not None:
                    self.A[i] = A[i]

        if B is not None:
            if len(B) != 3:
                raise ValueError("Parameter 'B' must be a list of three floats [xB, yB, zB]")
            for i in range(len(B)):
                if B[i] is not None:
                    self.B[i] = B[i]

        if V1 is not None:
            if not isinstance(V1, float):
                raise ValueError("Parameter 'V1' must be a float")
            self.V1 = V1

        if V2 is not None:
            if not isinstance(V2, float):
                raise ValueError("Parameter 'V2' must be a float")
            self.V2 = V2

    def get_trajectory_parameters(self, parameters=None):
        """
        Get trajectory parameters of the robot.

        Parameters:
            parameters : list of str or None, optional
                List of parameters to retrieve. If None, returns all parameters.
                Available parameters: 'A', 'B', 'V1', 'V2'.

        Returns:
            dict or any
                If parameters is None, returns a dictionary containing all trajectory parameters.
                If parameters is a list of parameter names, returns a dictionary containing only the specified parameters.
                If a single parameter is requested, returns the value of that parameter.
        """
        trajectory_params = {}

        if parameters is None:
            return {
                'A': self.A,
                'B': self.B,
                'V1': self.V1,
                'V2': self.V2
            }

        if isinstance(parameters, str):
            parameters = [parameters]

        for param in parameters:
            if param in ['A', 'B', 'V1', 'V2']:
                trajectory_params[param] = getattr(self, param)
            else:
                raise ValueError(f"Invalid parameter '{param}'")

        # If only one parameter is requested, return its value directly
        if len(trajectory_params) == 1:
            return list(trajectory_params.values())[0]

        return trajectory_params


    def trajectory_calculations(self, A=None, B=None, V1=None, V2=None):
        """
        Parameters:
            A : list[float] or None, optional
                Starting position of the trajectory [xA, yA, zA].
            B : list[float] or None, optional
                Ending position of the trajectory [xB, yB, zB].
            V1 : float or None, optional
                Initial velocity.
            V2 : float or None, optional
                Final velocity.
        
        Return :
            q0 :
            q1 :
        """
        # ========================
        # Robotic Design TP Code :
        # ========================
        # # Initialization
        # if A==None:A=self.A
        # if B==None:B=self.B
        # if V1==None:V1=self.V1
        # if V2==None:V2=self.V2

        # self.t = 0
        # s0, s1, s2 = [], [], []  # position, speed, acceleration
        # xt, yt, zt = [], [], []
        # self.Xs = [xt, yt, zt]
        # x0, y0, z0 = [], [], []  # x, y, z position
        # x1, y1, z1 = [], [], []  # x, y, z speed
        # x2, y2, z2 = [], [], []  # x, y, z acceleration
        # self.s, self.x, self.y, self.z = [s0, s1, s2], [x0, x1, x2], [y0, y1, y2], [z0, z1, z2]
        # self.endEffector_speed = []
        # q0, q1 = [], []

        # dAB = np.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2 + (A[2] - B[2]) ** 2)
        # self.T = dAB / (V2 + 2 * V1)
        # K1 = V1 / self.T
        # K2 = (V2 - V1) / self.T

        # # Calculate the laws at each sampling instant
        # self.t = np.linspace(0, 4 * self.T, 100)
        # for ti in self.t:
        #     if ti < self.T:
        #         s2.append(K1)
        #         s1.append(K1 * ti)
        #         s0.append((1 / 2) * K1 * ti ** 2)
        #     elif self.T <= ti < 2 * self.T:
        #         s2.append(K2)
        #         s1.append(K2 * (ti - self.T) + V1)
        #         s0.append((K2 / 2) * (ti - self.T) ** 2 + V1 * (ti - self.T) + (1 / 2) * K1 * self.T ** 2)
        #     elif 2 * self.T <= ti < 3 * self.T:
        #         s2.append(-K2)
        #         s1.append(-K2 * (ti - 2 * self.T) + V2)
        #         s0.append((-K2 / 2) * (ti - 2 * self.T) ** 2 + V2 * (ti - 2 * self.T) + ((self.T ** 2) / 2) * (K2 + K1) + V1 * self.T)
        #     elif ti >= 3 * self.T:
        #         s2.append(-K1)
        #         s1.append(-K1 * (ti - 3 * self.T) + V1)
        #         s0.append((-K1 / 2) * ((ti - 3 * self.T) ** 2) + V1 * (ti - 3 * self.T) + (self.T ** 2) / 2 * K1 + self.T * (V1 + V2))

        #     # Calculate the operational trajectory X(s) of line segments
        #     xt.append(((B[0]-A[0])/dAB)*s0[-1]+A[0])
        #     yt.append(((B[1]-A[1])/dAB)*s0[-1]+A[1])
        #     zt.append(((B[2]-A[2])/dAB)*s0[-1]+A[2])

        # for i in range(len(self.t)):
        #     # Calculate s(t), s*(t), and s**(t) for motion generation in the task space X(t)
        #     x0.append(((B[0] - A[0]) / dAB) * s0[i] + A[0])
        #     y0.append(((B[1] - A[1]) / dAB) * s0[i] + A[1])
        #     z0.append(((B[2] - A[2]) / dAB) * s0[i] + A[2])
        #     x1.append(((B[0] - A[0]) / dAB) * s1[i])
        #     y1.append(((B[1] - A[1]) / dAB) * s1[i])
        #     z1.append(((B[2] - A[2]) / dAB) * s1[i])
        #     x2.append(((B[0] - A[0]) / dAB) * s2[i])
        #     y2.append(((B[1] - A[1]) / dAB) * s2[i])
        #     z2.append(((B[2] - A[2]) / dAB) * s2[i])

        #     # Calculate the speed of the tool in the task space X(t)
        #     self.endEffector_speed.append(np.sqrt(x1[i] ** 2 + y1[i] ** 2 + z1[i] ** 2))

        #     # Calculate the q(t) and q_dot(t) trajectory for each joints
        #     # As we have not studied the singularity, we impose the first possibility [0]
        #     q0.append(self.inverse_kinematics(x0[i],y0[i],z0[i])[0])
        #     q1.append(self.inverse_dynamics(q0[i],[x1[i],y1[i],z1[i]]))
        
        # return q0,q1

    # DISPLAY ================================================================================================================
    def display_data(self,data,title,labels):
        """
        Display data on a graph.

        Parameters:
            data (list): List of data arrays to be plotted.
            title (str): Title of the graph.
            labels (list): List of labels for each data series.

        Returns:
            None
        """
        print(title)
        fig, axes = plt.subplots(1, len(labels), figsize=(5*len(labels), 3))
        for i in range(len(labels)):
            axes[i].plot(self.t, data[i], label=labels[i])
            axes[i].set_xlabel('time')
            axes[i].set_ylabel(labels[i])
            axes[i].legend()
            axes[i].set_title(f'{labels[i]} over time')

            # Add switch times as markers
            switch_times = [self.T, 2 * self.T, 3 * self.T, 4 * self.T]
            for switch_time in switch_times:
                axes[i].axvline(x=switch_time, color='r', linestyle='--', linewidth=1)

        # Print switch times on terminal
        print("switch_times\t=\t",switch_times)

        plt.tight_layout()
        plt.show()

    def display_s(self):
        """
        Display the graph of the imposed trajectory checking the evolution laws of s(t), s*(t), and s**(t)
        """
        self.display_data(self.s,"\nGraph of the imposed trajectory checking the laws of evolution s(t), s*(t) and s**(t)",["s","s*","s**"])

    def display_x(self):
        """
        Display the graph of the imposed trajectory checking the evolution laws of x(t), x*(t), and x**(t)
        """
        self.display_data(self.x,"\nGraph of the imposed trajectory checking the laws of evolution x(t), x*(t) et x**(t)",["x","x*","x**"])

    def display_y(self):
        """
        Display the graph of the imposed trajectory checking the evolution laws of y(t), y*(t), and y**(t)
        """
        self.display_data(self.y,"\nGraph of the imposed trajectory checking the laws of evolution y(t), y*(t) et y**(t)",["y","y*","y**"])

    def display_z(self):
        """
        Display the graph of the imposed trajectory checking the evolution laws of z(t), z*(t), and z**(t)
        """
        self.display_data(self.z,"\nGraph of the imposed trajectory checking the laws of evolution z(t), z*(t) et z**(t)",["z","z*","z**"])

    def display_endEffector_speed(self):
        """
        Display the speed profile of the tool against s*(t)
        """
        self.display_data([self.endEffector_speed,self.s[1]],"\nCheck speed profile of the tool against s*(t)",["endEffector_speed","s*"])

    def display_Xs(self):
        """
        Display the operational trajectory Xs in 3D
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(self.Xs[0], self.Xs[1], self.Xs[2], label='operational trajectory X(s)')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.show()
    
    def display_q_dotq(self, q, dotq):
        """
        Display joint positions q(t) and velocities dq(t) over time.

        Parameters:
            q (list): List of joint positions arrays.
            dotq (list): List of joint velocities arrays.

        Returns:
            None
        """
        num_joints = len(q[0])  # Assuming q[0] contains joint positions for a given time

        fig, axes = plt.subplots(num_joints, 2, figsize=(10, 3 * num_joints))

        for i in range(num_joints):
            axes[i, 0].plot(self.t, [q_t[i] for q_t in q], label=f'Joint {i + 1}')
            axes[i, 0].set_xlabel('Time')
            axes[i, 0].set_ylabel(f'q{i + 1}(t)')
            axes[i, 0].legend()
            axes[i, 0].set_title(f'Joint Position q{i + 1}(t)')

            axes[i, 1].plot(self.t, [dotq_t[i] for dotq_t in dotq], label=f'Joint {i + 1}')
            axes[i, 1].set_xlabel('Time')
            axes[i, 1].set_ylabel(f'dq{i + 1}(t)')
            axes[i, 1].legend()
            axes[i, 1].set_title(f'Joint Velocity dq{i + 1}(t)')

            # Add switch times as markers
            switch_times = [self.T, 2 * self.T, 3 * self.T, 4 * self.T]
            for switch_time in switch_times:
                axes[i, 0].axvline(x=switch_time, color='r', linestyle='--', linewidth=1)
                axes[i, 1].axvline(x=switch_time, color='r', linestyle='--', linewidth=1)

        # Print switch times on terminal
        print("Switch times\t=\t", switch_times)

        plt.tight_layout()
        plt.show()

    # MODEL CALCULATIONS ================================================================================================================
    def forward_kinematics(self, q):
        """
        Perform the forward kinematics (Geometric Direct Model) to compute the end-effector position from joint configurations.

        Parameters:
            q : list[float]
                Joint configurations for all joints.

        Returns:
            X : float
                X-coordinate of the end-effector position.
            Y : float
                Y-coordinate of the end-effector position.
            Z : float
                Z-coordinate of the end-effector position.
        """

        # ========================
        # Robotic Design TP Code :
        # =========================
        # for i in range(3):
        #     T = np.array([[np.cos(q[i]), -np.sin(q[i]), 0, self.a[i]],
        #                   [np.cos(self.alpha[i]) * np.sin(q[i]), np.cos(self.alpha[i]) * np.cos(q[i]), -np.sin(self.alpha[i]), -self.r[i] * np.sin(self.alpha[i])],
        #                   [np.sin(self.alpha[i]) * np.sin(q[i]), np.sin(self.alpha[i]) * np.cos(q[i]), np.cos(self.alpha[i]), self.r[i] * np.cos(self.alpha[i])],
        #                   [0, 0, 0, 1]])
        #     T0N = T0N @ T if i != 0 else T
        # T0E = T0N @ np.array([[1, 0, 0, self.la], [0, 1, 0, 0], [0, 0, 1, self.lb], [0, 0, 0, 1]])
        # X, Y, Z = T0E[:3, 3]
        # phi,nu,psi = [np.arctan2(-T0E[1, 2], T0E[2, 2])**(2), np.arcsin(T0E[0, 2]), np.arctan2(T0E[0, 1], T0E[0, 0])]
        # return X, Y, Z

    def inverse_kinematics(self, X, Y, Z):
        """
        Perform the inverse kinematics (Geometric Inverse Model) to compute joint configurations from the end-effector position.

        Parameters:
            X : float
                X-coordinate of the desired end-effector position.
            Y : float
                Y-coordinate of the desired end-effector position.
            Z : float
                Z-coordinate of the desired end-effector position.

        Returns:
            all_q : list[float]
                List of joint configurations for all joints corresponding to the given end-effector position.
        """

        # ========================
        # Robotic Design TP Code :
        # =========================
        # z1 = []
        # q1, q2, q3 = [], [], []
        # all_q = []

        # z1.append(np.sqrt(X**(2) + Y**(2) - ((self.lb - self.o1o2)**(2))))
        # z1.append(-np.sqrt(X**(2) + Y**(2) - ((self.lb - self.o1o2)**(2))))
        # z1.append(np.sqrt(X**(2) + Y**(2) - ((self.lb - self.o1o2)**(2))))
        # z1.append(-np.sqrt(X**(2) + Y**(2) - ((self.lb - self.o1o2)**(2))))
        # z2 = Z - self.o0o1
        # c3 = (z1[0]**(2) + z2**(2) - self.o2o3**(2) - self.la**(2)) / (2 * self.o2o3 * self.la)
        # q3.append(np.arctan2(np.sqrt(1 - c3**(2)), c3))
        # q3.append(np.arctan2(np.sqrt(1 - c3**(2)), c3))
        # q3.append(np.arctan2(-np.sqrt(1 - c3**(2)), c3))
        # q3.append(np.arctan2(-np.sqrt(1 - c3**(2)), c3))

        # for i in range (len(q3)):
        #     b1=self.o2o3+self.la*np.cos(q3[i])
        #     b2=self.la*np.sin(q3[i])

        #     s2=(b1*z2-b2*z1[i])/(b1**(2)+b2**(2))
        #     c2=(b1*z1[i]+b2*z2)/(b1**(2)+b2**(2))
        #     q2.append(np.arctan2(s2,c2))

        #     s1=(X*(-self.lb+self.o1o2)-Y*(self.la*np.cos(q2[i]+q3[i])+self.o2o3*np.cos(q2[i])))/((self.lb-self.o1o2)*(-self.lb+self.o1o2)-(self.la*np.cos(q2[i]+q3[i])+self.o2o3*np.cos(q2[i]))*(self.la*np.cos(q2[i]+q3[i])+self.o2o3*np.cos(q2[i])))
        #     c1=(Y*(self.lb-self.o1o2)-X*(self.la*np.cos(q2[i]+q3[i])+self.o2o3*np.cos(q2[i])))/((self.lb-self.o1o2)*(-self.lb+self.o1o2)-(self.la*np.cos(q2[i]+q3[i])+self.o2o3*np.cos(q2[i]))*(self.la*np.cos(q2[i]+q3[i])+self.o2o3*np.cos(q2[i])))
        #     q1.append(np.arctan2(s1,c1))

        #     # print([q1[i],q2[i],q3[i]])
        #     all_q.append([q1[i],q2[i],q3[i]])
        
        # return all_q

    def jacobian(self,q):
        """
        Compute the Jacobian matrix for the robot given its configurations.

        Parameters:
            q : list[float]
                Robot configurations.

        Returns:
            j : numpy.ndarray
                Jacobian matrix. 
        """
        # ========================
        # Robotic Design TP Code :
        # =========================
        # return np.array([   [-self.la*np.sin(q[0])*np.cos(q[1]+q[1])+self.lb*np.cos(q[0])-self.o1o2*np.cos(q[0])-self.o2o3*np.cos(q[1])*np.sin(q[0])    ,   -self.la*np.cos(q[0])*np.sin(q[1]+q[2])-self.o2o3*np.cos(q[0])*np.sin(q[1]) ,   -self.la*np.cos(q[0])*np.sin(q[1]+q[2])  ], 
        #                 [ self.la*np.cos(q[0])*np.cos(q[1]+q[2])+(self.o0o1+self.o1o2-self.o2o3)*np.sin(q[0])-self.o2o3*np.cos(q[1])*np.cos(q[0])   ,   -self.la*np.sin(q[0])*np.sin(q[1]-q[2])-self.o2o3*np.sin(q[1])*np.sin(q[0]) ,   self.la*np.sin(q[0])*np.sin(q[1]-q[2])   ],
        #                 [ 0                                                                                                                         ,   self.la*np.cos(q[1]+q[2])+self.o2o3*np.cos(q[1])                            ,   self.la*np.cos(q[2]+q[1])                ]])
        
    def inverse_dynamics (self, q, dotX):
        """
        Perform the model-based inverse dynamics computation.

        Parameters:
            q : list[float]
                Robot configurations.
            dotX : numpy.ndarray
                Transpose of the velocity vector.

        Returns:
            qdot : numpy.ndarray
                Inverse of the Jacobian multiplied by the transpose of the velocity vector.
        """
        # ========================
        # Robotic Design TP Code :
        # =========================
        # return np.dot(np.linalg.inv(self.jacobian(q)),np.transpose(dotX))
