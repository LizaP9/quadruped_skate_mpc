#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from math import pi
import json
import os
from inverse_kinematics import InverseKinematics

def run_tests():
    """Run a series of tests for the InverseKinematics solver"""
    # Initialize IK solver with default A1 dimensions
    ik_solver = InverseKinematics()
    
    # Define standard foot positions for A1 in neutral stance
    # [FR, FL, RR, RL] - each is [x, y, z] in the world frame
    default_stance = np.array([
        [0.1838, -0.147, 0.05],    # FR
        [0.1838,  0.147, 0.05],    # FL
        [-0.1838, -0.147, 0.05],   # RR
        [-0.1838,  0.147, 0.05]    # RL
    ])
    
    # Default body position and orientation
    default_pos = [0.0, 0.1, 0.3]
    default_ori = [0.0, 0.0, 0.0]
    
    # Test cases
    test_cases = [
        {
            "name": "default_stance",
            "feet_positions": default_stance,
            "position": default_pos,
            "orientation": default_ori
        },
        {
            "name": "shift_forward",
            "feet_positions": default_stance,
            "position": [0.05, 0.0, 0.3],  # Shifted 5cm forward
            "orientation": default_ori
        },
        {
            "name": "shift_left",
            "feet_positions": default_stance,
            "position": [0.0, 0.05, 0.3],  # Shifted 5cm to the left
            "orientation": default_ori
        },
        {
            "name": "lower_body",
            "feet_positions": default_stance,
            "position": [0.0, 0.0, 0.25],  # Lowered 5cm
            "orientation": default_ori
        },
        {
            "name": "roll_right",
            "feet_positions": default_stance,
            "position": default_pos,
            "orientation": [0.2, 0.0, 0.0]  # Roll 0.2 rad to the right
        },
        {
            "name": "pitch_down",
            "feet_positions": default_stance,
            "position": default_pos,
            "orientation": [0.0, 0.2, 0.0]  # Pitch 0.2 rad down
        },
        {
            "name": "yaw_left",
            "feet_positions": default_stance,
            "position": default_pos,
            "orientation": [0.0, 0.0, 0.2]  # Yaw 0.2 rad to the left
        },
        {
            "name": "combined",
            "feet_positions": default_stance,
            "position": [0.03, 0.03, 0.28],  # Shifted and lowered
            "orientation": [0.1, 0.1, 0.1]   # Combined rotation
        },
        {
            "name": "on_skate",
            "feet_positions": np.array([
                [0.1838, -0.1, 0.1],    # FR
                [0.1838,  0.1, 0.1],    # FL
                [-0.1838, -0.1, 0.1],   # RR
                [-0.1838,  0.1, 0.1]    # RL
            ]),
            "position": default_pos,  # Shifted and lowered
            "orientation": default_ori  # Combined rotation
        },
        {
            "name": "on_skate_rigth_back_shift",
            "feet_positions": np.array([
                [0.1838, -0.1, 0.1],    # FR
                [0.1838,  0.1, 0.1],    # FL
                [-0.1838, -0.1, 0.1],   # RR
                [-0.1838,  0.1, 0.1]    # RL
            ]),
            "position": [-0.07, -0.05, 0.3],  # Shifted and lowered
            "orientation": default_ori  # Combined rotation
        },
        {
            "name": "on_skate_rigth_back_shift_left_leg_center",
            "feet_positions": np.array([
                [0.1838, -0.1, 0.1],    # FR
                [0.1838,  0, 0.1],    # FL
                [-0.1838, -0.1, 0.1],   # RR
                [-0.1838,  0.1, 0.1]    # RL
            ]),
            "position": [-0.07, -0.05, 0.3],  # Shifted and lowered
            "orientation": default_ori  # Combined rotation
        },
        {
            "name": "on_skate_left_leg_center",
            "feet_positions": np.array([
                [0.1838, -0.1, 0.1],    # FR
                [0.1838,  0, 0.1],    # FL
                [-0.1838, -0.1, 0.1],   # RR
                [-0.1838,  0.1, 0.1]    # RL
            ]),
            "position": default_pos,  # Shifted and lowered
            "orientation": default_ori  # Combined rotation
        },
        {
            "name": "on_skate_ready_to_go",
            "feet_positions": np.array([
                [0.1838, -0.2, 0.1],    # FR
                [0.1838,  0, 0.1],    # FL
                [-0.1838, -0.1, 0.1],   # RR
                [-0.1838,  0.1, 0.1]    # RL
            ]),
            "position": default_pos,  # Shifted and lowered
            "orientation": default_ori  # Combined rotation
        },
    ]
    
    # Run tests and collect results
    results = {}
    for test in test_cases:
        print(f"Running test: {test['name']}")
        
        # Solve inverse kinematics
        joint_angles = ik_solver.solve(
            test['feet_positions'], 
            test['position'], 
            test['orientation']
        )
        
        # Store results
        results[test['name']] = {
            "feet_positions": test['feet_positions'].tolist(),
            "position": test['position'],
            "orientation": test['orientation'],
            "joint_angles": joint_angles
        }
        
        # Print results
        print(f"  Joint angles: {np.array(joint_angles) * 180/pi} degrees")
        print()
    
    # Save results to file for visualization
    results_dir = os.path.dirname(os.path.abspath(__file__))
    results_file = os.path.join(results_dir, "ik_test_results.json")
    with open(results_file, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"Test results saved to {results_file}")
    return results

if __name__ == "__main__":
    results = run_tests()