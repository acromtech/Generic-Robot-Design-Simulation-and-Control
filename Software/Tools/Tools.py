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
import re
import time
import numpy as np
import matplotlib.pyplot as plt

PRISMATIC = 1
REVOLUTE = 0
X, Y, Z = 0, 1, 2

class Tools:

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

    def __init__(self, 
                 joint_types=[], joint_distances=[], joint_orientations=[], 
                 alpha=[], d=[], theta=[], a=[]):
        """
        Initialize the Tools class.

        Parameters:
            joint_types         list[int]      List of joint types (1=Prismatic, 0=Revolute).       Defaults to an empty list.
            joint_distances     list[float]    List of distances between each joints (in m).        Defaults to an empty list.
            joint_orientations  list[int]      List of orientations of each joints (0=X, 1=Y, 2=Z). Defaults to an empty list.

            alpha               list[float]     List of alphaj DHM parameter     -> ang(zj-1,zj) autour de xj-1      - ex : [alpha1,alpha2,alpha3,...]
            d                   list[float]     List of dj DHM parameter         -> dist(zj-1,zj) le long de xj-1    - ex : [a1,a2,a3,...]
            theta               list[float]     List of thetaj DHM parameter     -> ang(xj-1,xj) autour de zj        - ex : [theta1,theta2,theta3,...]
            r                   list[float]     List of rj DHM parameter         -> dist(xj-1,xj) le long de zj      - ex : [a1,a2,a3,...]

        Initializes the joint types, joint distances, joint orientations, and q configurations 
        based on the provided parameters. Additionally, calculates the DHM parameters based on 
        the initialized joint types, joint distances, and joint orientations.
        """
        self.joint_types = joint_types
        self.joint_distances = joint_distances
        self.joint_orientations = joint_orientations

        self.num_joints = len(joint_types)
        if d is None and alpha is None and a is None and theta is None : 
            self.calculate_DHM_parameters()
        else:
            self.d = d
            self.alpha = alpha
            self.a = a
            self.theta = theta

    # DHM PARAMETERS ================================================================================================================
    def calculate_DHM_parameters(self):
        """
        Calculate DHM parameters for each joints of the robot features

        Parameters :
            joint_types          :list[int]              # List of joint type (1=Prismatic, 0=Revolute)                      - ex : Robot RPPR -> [0,1,1,0]
            joint_distances      :list[float]            # List of contants - distance beetween each joints (in m)           - ex : [o0o1,o1o2,o2o3,...]
            joint_orientations   :list[int]              # List of contants - orientation of each joints (0=X, 1=Y, 2=Z)     - ex : [0,0,1,...]

        Return :
            alpha = []  # List of alphaj DHM parameter     -> ang(zj-1,zj) autour de xj-1      - ex : [alpha1,alpha2,alpha3,...]
            d = []      # List of dj DHM parameter         -> dist(zj-1,zj) le long de xj-1    - ex : [d1,d2,d3,...]
            theta = []  # List of thetaj DHM parameter     -> ang(xj-1,xj) autour de zj        - ex : [theta1,theta2,theta3,...]
            a = []      # List of aj DHM parameter         -> dist(xj-1,xj) le long de zj      - ex : [a1,a2,a3,...]
        """
        
    
    def set_DHM_parameters(self, a=None, alpha=None, d=None, theta=None):
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

        if d is not None:
            if len(d) != len(self.joint_distances):
                raise ValueError("Length of parameter 'd' does not match the current number of joints")
            for i in range(len(d)):
                if d[i] is not None:
                    self.d[i] = d[i]

        if theta is not None:
            if len(theta) != len(self.joint_distances):
                raise ValueError("Length of parameter 'theta' does not match the current number of joints")
            for i in range(len(theta)):
                if theta[i] is not None:
                    self.theta[i] = theta[i]

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
                    'd': self.d,
                    'alpha': self.alpha,
                    'r': self.r
                    'theta' : self.theta
                }
        """
        parameters = {
            'd': self.d,
            'alpha': self.alpha,
            'a': self.a,
            'theta' : self.theta
        }

        if parameter is not None:
            if parameter not in parameters:
                raise ValueError("Invalid parameter name")
            return parameters[parameter]
        else:
            return parameters

    # JOINT PARAMETERS ================================================================================================================
    def set_joint_parameters(self, joint_types=None, joint_distances=None, joint_orientations=None):
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

        q1,q2,q3,q4 = q

        #Definition des longueurs
        l1 = self.joint_distances[0]
        l2 = self.joint_distances[1]
        L1 = self.joint_distances[2]
        L4 = self.joint_distances[3]

        c1 = np.cos(q1)
        s1 = np.sin(q1)
        c4 = np.cos(q4)
        s4 = np.sin(q4)
        c3 = np.cos(q3)
        s3 = np.sin(q3)

        T04 = np.array([[c4*(c1*c3 - s1*s3) + s4*(-c1*s3 - c3*s1),        c4*(-c1*s3 - c3*s1) - s4*(c1*c3 - s1*s3),         0,            c1*l1 + l2*(c1*c3 - s1*s3)],
            [c4*(c1*s3 + c3*s1) + s4*(c1*c3 - s1*s3),         c4*(c1*c3 - s1*s3) - s4*(c1*s3 + c3*s1),          0,            l1*s1 + l2*(c1*s3 + c3*s1)],
            [              0,                                                   0,                              1,                      L1 + q2         ],
            [              0,                                                   0,                              0,                        1             ]])
        
        R04 = T04[0:3,0:3]
        P = T04[0:3,3]

        P04 = np.array([[P[0]], [P[1]], [P[2]]])

        #Ot dans le repère R4
        Ot_R4 = [[0],[0],[-L4]]

        # Ot dans le repère R0
        Ot_R0 = np.dot(R04,Ot_R4)

        #Position organe terminal
        X = P04+Ot_R0
        
        #Angle organe terminal
        theta_d = np.array([[q1+q3+q4]])
        
        #Position et orientation de l'organe terminal
        Xd = np.vstack((X, theta_d )) 

        return Xd.T

    def inverse_kinematics(self, Xbut):
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
        # Lenght definition
        l1 = self.joint_distances[0]
        l2 = self.joint_distances[1]
        L1 = self.joint_distances[2]
        L4 = self.joint_distances[3]

        x = Xbut[0]
        y = Xbut[1]
        z = Xbut[2]
        theta_but = Xbut[3]

        q = []  # joint configuration
        
        #Déduction q2 : Ligne 3, Colonne 3 MGD 
        q2 = z-L1+L4
        #q2 = -z+L1-L4

        #Déduction q3 et q1 : Ligne 2 et 1, Colonne 3 MGD 
        
            # Notations utilisées : formules générales du cours :
            
            #_________Déduction q3_________
        
        
        Z1 = x 
        Z2 = y 

        signe_s3 = [-1,1] # possible signs of s3

        for signe in signe_s3 : 
            c3 = (Z1**2 + Z2**2 - l1**2 - l2**2 )/(2*l1*l2)
            s3 = signe*np.sqrt(1-c3**2)
            q3 = np.arctan2(s3, c3)
            B1 = l1 + l2*c3
            B2 = l2*s3
            s1 = (B1*Z2-B2*Z1)/(B1**2+B2**2)
            c1 = (B1*Z1+B2*Z2)/(B1**2+B2**2)
            q1 = (np.arctan2(s1, c1)) 

            q4 = theta_but-q1-q3

            q.append((q1,q2,q3,q4))

        return np.array(q)
        
    def jac_analytique(self, qdeg) :
        """
        Compute the analytique Jacobian matrix for the robot given its configurations.

        Parameters:
            q : list[float]
                Robot configurations.

        Returns:
            j : numpy.ndarray
                analytique Jacobian matrix. 
        """
        q = qdeg.copy()
        q[0] = np.deg2rad(q[0]) 
        q[2] = np.deg2rad(q[2])  
        q[3] = np.deg2rad(q[3])  
        q1,q2,q3,q4 = q
        
        l1 = self.joint_distances[0]
        l2= self.joint_distances[1]
        
        c1= np.cos(q1)
        s1=np.sin(q1)
        c4= np.cos(q4)
        s4=np.sin(q4)
        c3= np.cos(q3)
        s3=np.sin(q3)
        Ja=np.array([[-l1*s1-l2*(c3*s1+s3*c1),       0      ,       -l2*(c3*s1+s3*c1) ,       0     ], 
                    [l1*c1+l2*(c3*c1-s3*s1) ,       0      ,        l2*(c3*c1+s3*s1) ,       0     ], 
                    [          0            ,       1      ,               0         ,       0     ],
                    [          1            ,       0      ,               1         ,       1     ]
                    ])
        return Ja
    
    def jac_geo(self, qdeg) :
        """
        Compute the geometrique Jacobian matrix for the robot given its configurations.

        Parameters:
            q : list[float]
                Robot configurations.

        Returns:
            j : numpy.ndarray
                geometrique Jacobian matrix. 
        """
        q = qdeg.copy()
        q[0] = np.deg2rad(q[0]) 
        q[2] = np.deg2rad(q[2])  
        q[3] = np.deg2rad(q[3])  
        q1,q2,q3,q4 = q

        l1 = self.joint_distances[0]
        l2= self.joint_distances[1]
        
        c1= np.cos(q1)
        s1=np.sin(q1)
        c4= np.cos(q4)
        s4=np.sin(q4)
        c3= np.cos(q3)
        s3=np.sin(q3)

        J=np.array([[-l1*s1-l2*(c3*s1+s3*c1) ,       0      ,       -l2*(c3*s1+s3*c1) ,       0     ], 
                    [l1*c1+l2*(c3*c1-s3*s1) ,       0      ,        l2*(c3*c1+s3*s1) ,       0     ], 
                    [          0            ,       1      ,               0         ,       0     ],
                    [          1            ,       0      ,               1         ,       1     ]
                    ])
        return J
    
    def Direct_differential_kinematics(self,qp,q) :
        """
        Compute the differential kinematics (Differential Direct Model) to find the end-effector velocity from joint velocities.

        Parameters:
            qp : list[float]
                Joint velocities for all joints.
            q : list[float]
                Joint configurations for all joints.

        Returns:
            Xp : numpy.ndarray
                End-effector velocity.
        """
        q1,q2,q3,q4= q
        Ja = self.jac_analytique(q)
        Xp = np.dot(Ja,qp)
        return Xp
    
    def Indirect_differentiel_kinematics(self, Xp,q) :
        """
        Compute the joint velocities from end-effector velocity using the inverse differential kinematics.

        Parameters:
            Xp : numpy.ndarray
                End-effector velocity.
            q : list[float]
                Joint configurations for all joints.

        Returns:
            qp : numpy.ndarray
                Joint velocities for all joints.
        """
        q1,q2,q3,q4 = q
        Ja_inv = np.linalg.inv(self.jac_analytique(q))
        print("Jacc",Ja_inv)
        qp = np.dot(Ja_inv,Xp)
        return qp


    def extract_trajectory(self, file_path):
        with open(file_path, 'r') as file:
            data = file.read()

        matches = re.findall(r"positions: \[(.*?)\]\n\s+velocities: \[(.*?)\]\n\s+accelerations: \[(.*?)\]\n\s+effort: (.*?)\n\s+time_from_start:\s+\n\s+secs: (\d+)\n\s+nsecs:\s+(\d+)", data, re.DOTALL)

        configurations = []
        for match in matches:
            positions = [float(x) for x in match[0].split(',')]
            velocities = [float(x) for x in match[1].split(',')]
            accelerations = [float(x) for x in match[2].split(',')]
            effort = [] if match[3].strip() == "[]" else match[3].strip().split(',')
            secs = int(match[4])
            nsecs = int(match[5])

            information = {
                "positions": positions,
                "velocities": velocities,
                "accelerations": accelerations,
                "effort": effort,
                "time_from_start": {
                    "secs": secs,
                    "nsecs": nsecs
                }
            }
            configurations.append(information)
        return configurations

