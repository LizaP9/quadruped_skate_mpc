#!/usr/bin/env python3
import numpy as np
from math import sqrt, atan2, sin, cos, pi
from transformations import homog_transform, homog_transform_inverse

class InverseKinematics:
    """
    Inverse Kinematics solver for the Unitree A1 quadruped robot.
    
    This class implements the analytical inverse kinematics solution for a quadruped
    robot with a specific leg configuration (3 DoF per leg).
    """
    
    # Default dimensions for A1 robot (in meters)
    DEFAULT_BODY_DIMENSIONS = [0.366, 0.094]
    DEFAULT_LEG_DIMENSIONS = [0.,0.08505, 0.2, 0.2] 
    
    def __init__(self, body_dimensions=None, leg_dimensions=None):
        """
        Initialize the inverse kinematics solver with robot dimensions.
        
        Args:
            body_dimensions: List [length, width] of the robot body in meters
            leg_dimensions: List [hip_length, hip_offset, thigh_length, calf_length] in meters
        """
        # Body dimensions
        if body_dimensions is None:
            body_dimensions = self.DEFAULT_BODY_DIMENSIONS
            
        self.body_length = body_dimensions[0]
        self.body_width = body_dimensions[1]

        # Leg dimensions
        if leg_dimensions is None:
            leg_dimensions = self.DEFAULT_LEG_DIMENSIONS
            
        self.l1 = leg_dimensions[0]  # Hip length
        self.l2 = leg_dimensions[1]  # Hip offset
        self.l3 = leg_dimensions[2]  # Thigh length
        self.l4 = leg_dimensions[3]  # Calf length

    def solve(self, feet_positions, position, orientation):
        """
        Solve the inverse kinematics for all legs.
        
        Args:
            feet_positions: 4x3 array of foot positions in world frame [FR, FL, RR, RL]
            position: [x, y, z] body position in world frame
            orientation: [roll, pitch, yaw] body orientation in world frame (in radians)
            
        Returns:
            List of 12 joint angles (in radians) in order [FR_hip, FR_thigh, FR_calf, 
                                                         FL_hip, FL_thigh, FL_calf,
                                                         RR_hip, RR_thigh, RR_calf,
                                                         RL_hip, RL_thigh, RL_calf]
        """
        dx, dy, dz = position
        roll, pitch, yaw = orientation
        
        return self._inverse_kinematics(feet_positions, dx, dy, dz, roll, pitch, yaw)
    
    def _get_local_positions(self, leg_positions, dx, dy, dz, roll, pitch, yaw):
        """
        Compute the positions of the feet in the shoulder frames.
        
        Args:
            leg_positions: 4x3 array of foot positions in world frame
            dx, dy, dz: Body position
            roll, pitch, yaw: Body orientation
            
        Returns:
            4x3 array of foot positions in their respective shoulder frames
        """
        leg_positions = (np.block([[leg_positions.T],[np.array([1,1,1,1])]])).T

        # Transformation matrix, base_link_world => base_link
        T_blwbl = homog_transform(dx,dy,dz,roll,pitch,yaw)

        # Transformation matrix, base_link_world => FR1
        T_blwFR1 = np.dot(T_blwbl, homog_transform(+0.5*self.body_length,
                          -0.5*self.body_width,0,pi/2,-pi/2,0))

        # Transformation matrix, base_link_world => FL1
        T_blwFL1 = np.dot(T_blwbl, homog_transform(+0.5*self.body_length,
                          +0.5*self.body_width,0,pi/2,-pi/2,0))

        # Transformation matrix, base_link_world => RR1
        T_blwRR1 = np.dot(T_blwbl, homog_transform(-0.5*self.body_length,
                          -0.5*self.body_width,0,pi/2,-pi/2,0))

        # Transformation matrix, base_link_world => RL1
        T_blwRL1 = np.dot(T_blwbl, homog_transform(-0.5*self.body_length,
                          +0.5*self.body_width,0,pi/2,-pi/2,0))

        # Local coordinates
        pos_FR = np.dot(homog_transform_inverse(T_blwFR1),leg_positions[0])
        pos_FL = np.dot(homog_transform_inverse(T_blwFL1),leg_positions[1])
        pos_RR = np.dot(homog_transform_inverse(T_blwRR1),leg_positions[2])
        pos_RL = np.dot(homog_transform_inverse(T_blwRL1),leg_positions[3])

        return(np.array([pos_FR[:3],pos_FL[:3],pos_RR[:3],pos_RL[:3]]))

    def _inverse_kinematics(self, leg_positions, dx, dy, dz, roll, pitch, yaw):
        """
        Compute inverse kinematics for all legs.
        
        Args:
            leg_positions: 4x3 array of foot positions in world frame
            dx, dy, dz: Body position
            roll, pitch, yaw: Body orientation
            
        Returns:
            List of 12 joint angles (in radians)
        """
        # Get foot positions in shoulder frames
        positions = self._get_local_positions(leg_positions, dx, dy, dz, roll, pitch, yaw)
        
        # Container for joint angles
        angles = []

        # Solve IK for each leg
        # for i in range(4):
        #     y = positions[i][0]
        #     z = positions[i][1]
        #     x = positions[i][2]
            
        #     print(x, y, z)

        #     # Leg-specific sign for hip joint (depends on which side the leg is on)
        #     leg_sign = (-1)**((i % 2) == 1)  # -1 for FL, RL; +1 for FR, RR
            
        #     # Compute hip joint angle (abduction/adduction)
        #     F = sqrt(x**2 + y**2 - self.l2**2)
        #     G = F - self.l1
        #     H = sqrt(G**2 + z**2)
            
        #     # Hip joint (abduction/adduction)
        #     theta1 = -atan2(y, x) - atan2(F, self.l2 * leg_sign)
 
        #     # For the knee, compute angle using law of cosines
        #     D = (H**2 - self.l3**2 - self.l4**2) / (2 * self.l3 * self.l4)
        #     D = np.clip(D, -1.0, 1.0)  # Ensure in valid range for acos
            
        #     # Knee joint
        #     theta3 = atan2(sqrt(1 - D**2), D)
            
        #     # Thigh joint
        #     theta2 = atan2(z, G) - atan2(self.l4 * sin(theta3), 
        #                                 self.l3 + self.l4 * cos(theta3))

        #     # Account for joint direction conventions
        #     theta3 = -theta3  # Knee joint moves in opposite direction
            
        #     # Add to joint angles list
        #     angles.extend([theta1, theta2, theta3])
       
        # # Return joint angles in radians - FR, FL, RR, RL
        # return angles

        for i in range(4):
    
            x = positions[i][0]
            y = positions[i][1]
            z = positions[i][2]

            print(x, y, z)

            F = sqrt(x**2 + y**2 - self.l2**2)
            G = F - self.l1
            H = sqrt(G**2 + z**2)

            theta1 = -atan2(y,x) - atan2(F,self.l2 * (-1)**i)
 
            D = (H**2 - self.l3**2 - self.l4**2)/(2*self.l3*self.l4)

            theta4 = -atan2((sqrt(1-D**2)),D)

            theta3 = atan2(z,G) - atan2(self.l4*sin(theta4),
                                          self.l3 + self.l4*cos(theta4))

            angles.append(theta1)
            angles.append(theta3)
            angles.append(theta4)
       
        # Return joint angles in radians - FR, FL, RR, RL
        return angles
