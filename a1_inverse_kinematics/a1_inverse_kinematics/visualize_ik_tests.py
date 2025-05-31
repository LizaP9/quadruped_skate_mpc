#!/usr/bin/env python3
import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import json
import time
import argparse
from math import pi

def load_test_results(file_path):
    """Load test results from a JSON file"""
    with open(file_path, 'r') as f:
        return json.load(f)

def keyboard(window, key, scancode, act, mods):
    global paused, current_test_idx, test_results, test_names
    
    if act == glfw.PRESS:
        if key == glfw.KEY_BACKSPACE:
            mj.mj_resetData(model, data)
            mj.mj_forward(model, data)
        elif key == glfw.KEY_SPACE:
            paused = not paused
        elif key == glfw.KEY_RIGHT:
            current_test_idx = (current_test_idx + 1) % len(test_names)
            print(f"Showing test: {test_names[current_test_idx]}")
            load_test_pose(test_results[test_names[current_test_idx]])
        elif key == glfw.KEY_LEFT:
            current_test_idx = (current_test_idx - 1) % len(test_names)
            print(f"Showing test: {test_names[current_test_idx]}")
            load_test_pose(test_results[test_names[current_test_idx]])

def mouse_button(window, button, act, mods):
    # update button state
    global button_left, button_middle, button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx, lasty, button_left, button_middle, button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

def load_test_pose(test_data):
    """
    Load a specific test pose into the MuJoCo simulation
    
    Args:
        test_data: Dictionary containing the test data (position, orientation, joint_angles)
    """
    # Extract test data
    position = test_data["position"]
    orientation = test_data["orientation"]
    joint_angles = test_data["joint_angles"]
    
    # Convert Euler angles to quaternion for MuJoCo
    # MuJoCo uses quaternion for orientation
    # This is a simple conversion for small angles, for real application consider using a proper library
    roll, pitch, yaw = orientation
    
    # Simple conversion from Euler angles to quaternion (for small angles)
    cr, cp, cy = np.cos(roll/2), np.cos(pitch/2), np.cos(yaw/2)
    sr, sp, sy = np.sin(roll/2), np.sin(pitch/2), np.sin(yaw/2)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    quat = np.array([w, x, y, z])
    
    # Create the position state for the A1 robot
    # Position state for A1: [x, y, z, qw, qx, qy, qz, 12 joint angles]
    pos_state = np.zeros(19)
    pos_state[0:3] = position  # x, y, z
    pos_state[3:7] = quat  # qw, qx, qy, qz
    pos_state[7:19] = joint_angles  # 12 joint angles
    
    # Set the position state in MuJoCo
    data.qpos[:19] = pos_state


    #model.dof_damping[:] = 1000 # PREVENT GRAVITY 
    
    # Update the physics simulation
    mj.mj_forward(model, data)

def render_pose_info(window, test_name):
    """Render the current pose information on the screen"""
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    
    # Create text overlay
    text_overlay = f"Test: {test_name}\n"
    text_overlay += "Left/Right: Change pose\n"
    text_overlay += "Space: Pause/Resume"
    
    mj.mjr_overlay(mj.mjtFont.mjFONT_NORMAL, mj.mjtGridPos.mjGRID_BOTTOMLEFT, 
                  viewport, text_overlay, None, context)

# Main function
def main():
    global model, data, scene, cam, context
    global button_left, button_middle, button_right, lastx, lasty
    global paused, current_test_idx, test_results, test_names
    
    parser = argparse.ArgumentParser(description='Visualize IK Test Results')
    parser.add_argument('--results', type=str, default=None,
                        help='Path to the IK test results JSON file')
    args = parser.parse_args()
    
    # If no results file is provided, use the default one
    if args.results is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        args.results = os.path.join(script_dir, "ik_test_results.json")
    
    # Load test results
    test_results = load_test_results(args.results)
    test_names = list(test_results.keys())
    current_test_idx = 0
    paused = True
    
    # For callback functions
    button_left = False
    button_middle = False
    button_right = False
    lastx = 0
    lasty = 0
    
    # Path to scene.xml
    pkg_path = os.path.dirname(os.path.dirname(__file__))
    scene_path = os.path.join(pkg_path, "../a1_description/scene.xml")
    scene_path = os.path.abspath(scene_path)
    
    if not os.path.exists(scene_path):
        raise FileNotFoundError(f"Scene file not found: {scene_path}")
    
    # MuJoCo initialization
    model = mj.MjModel.from_xml_path(scene_path)
    data = mj.MjData(model)
    
    # Initialize GLFW
    glfw.init()
    window = glfw.create_window(1200, 900, "A1 IK Test Viewer", None, None)
    glfw.make_context_current(window)
    glfw.swap_interval(1)
    
    # Camera setup
    cam = mj.MjvCamera()
    opt = mj.MjvOption()
    mj.mjv_defaultCamera(cam)
    mj.mjv_defaultOption(opt)
    
    scene = mj.MjvScene(model, maxgeom=10000)
    context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)
    
    # Set callbacks
    glfw.set_key_callback(window, keyboard)
    glfw.set_cursor_pos_callback(window, mouse_move)
    glfw.set_mouse_button_callback(window, mouse_button)
    glfw.set_scroll_callback(window, scroll)
    
    # Load initial test pose
    load_test_pose(test_results[test_names[current_test_idx]])
    print(f"Loaded test: {test_names[current_test_idx]}")
    print("Press Space to pause/resume, Left/Right arrows to cycle through tests")
    
    # Main simulation loop
    while not glfw.window_should_close(window):
        time_prev = data.time
        
        if not paused:
            # Run physics for a short period
            while (data.time - time_prev < 1.0/60.0):
                mj.mj_step(model, data)
        
        # Render scene
        viewport_width, viewport_height = glfw.get_framebuffer_size(window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
        
        # Update camera target to follow the robot
        cam.lookat[0] = data.qpos[0]
        cam.lookat[1] = data.qpos[1]
        cam.lookat[2] = data.qpos[2]
        
        # Update scene and render
        mj.mjv_updateScene(model, data, opt, None, cam,
                          mj.mjtCatBit.mjCAT_ALL.value, scene)
        mj.mjr_render(viewport, scene, context)
        
        # Overlay pose information
        render_pose_info(window, test_names[current_test_idx])
        
        # Swap OpenGL buffers
        glfw.swap_buffers(window)
        
        # Process events
        glfw.poll_events()
    
    # Cleanup
    glfw.terminate()

if __name__ == "__main__":
    main()