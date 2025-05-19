#!/usr/bin/env python3
import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os

import rclpy
from rclpy.node import Node
from quadruped_skate_mpc.msg import States
from geometry_msgs.msg import Pose, Twist


rclpy.init()
node = rclpy.create_node('a1_sim_state_publisher')

joint_pub = node.create_publisher(States, '/a1/joint_act_states', 10)
com_pub = node.create_publisher(Pose, '/a1/com_act_state', 10)



# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    pass

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

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
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

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
    

# Path to scene.xml
pkg_path = os.path.dirname(__file__)
scene_path = os.path.join(pkg_path, "../a1_description/scene.xml")
scene_path = os.path.abspath(scene_path)

if not os.path.exists(scene_path):
    raise FileNotFoundError(f"Scene file not found: {scene_path}")

# MuJoCo
model = mj.MjModel.from_xml_path(scene_path)
data = mj.MjData(model)

# Camera, window
glfw.init()
window = glfw.create_window(1200, 900, "A1 Viewer", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

cam = mj.MjvCamera()
opt = mj.MjvOption()
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)

scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)



glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

#initialize the controller
init_controller(model,data)

#set the controller
mj.set_mjcb_control(controller)

hip_r = 0.0
hip_l = 0.0

# stance on skate
# hip_r = 0.17
# hip_l = -0.17

pitch = 0.9
knee = -1.8

pos = np.array([0, 0.0, 0.37])
quat = np.array([1,0,0,0])
#euler = np.array([0,0,np.pi/2])
#quat = ram.bryant2quat(euler)

qleg_r = np.array([hip_r,pitch,knee])
qleg_l = np.array([hip_l,pitch,knee])


# position for a1 - first 19 elements
data.qpos[:19] = np.concatenate((pos,quat,qleg_r,qleg_l,qleg_r,qleg_l))

# ctrl = np.array([0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8])


# MuJoCo doesn't allow multiple .xml file, so you need to add everything in one .xml.
# Thus, a1 + skate is consedered as one robot and have combines state and all of it's joints can be controller.
#  In our case it's "scene.xml". The order to include a1.xml and skate.xml metter to understand the state

# For a1 + skate:

# State:
# position 1x32 - a1 CoM linear position (3), a1 CoM quaternion (4), a1 joints (motors) (12), 
# skate CoM linear position (3), skate CoM quaternion (4), skate joints (6)

# velocity 1x30 - a1 CoM linear vel (3), a1 angular vel (3), a1 joint vel (motors) (12), 
# skate CoM linear vel (3), skate angular vel (3), skate joint vel (6)


# control - 1x32 - same as position state for position control

while not glfw.window_should_close(window):
    time_prev = data.time

    #Loop to run several control steps and status updates within a single simulation frame
    while (data.time - time_prev < 1.0/15.0):
        # mj.mj_step1(model, data)
        # Обновление состояний
        q_act = data.qpos[7:19].copy()
        v_act = data.qvel[6:18].copy()
        pos_quat_trunk = data.qpos[:7].copy()
        vel_angvel_trunk = data.qvel[:6].copy()

        # a1 JointState msg
        js = States()

        js.t = data.time 
        js.qj = q_act.tolist()
        js.vj = v_act.tolist()
        js.tauj = data.ctrl[:12].tolist()
        # js.tauj = [0.0] * 12

        # quaternion
        js.imu_orientation = data.qpos[3:7].tolist()
        js.imu_angular_velocity = data.qvel[3:6].tolist()
        js.imu_linear_acceleration = data.qacc[:3].tolist()  # или [0,0,0] если нет акселерометра


        foot_force_sensor = [0.0, 0.0, 0.0, 0.0]

        foot_geom_names = ['FR_foot_geom', 'FL_foot_geom', 'RR_foot_geom', 'RL_foot_geom']
        foot_geom_ids = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, geom_name) for geom_name in foot_geom_names]

        for i in range(data.ncon):
            contact = data.contact[i]
            force = np.zeros(6)
            mj.mj_contactForce(model, data, i, force)
            
            # Check if contact involves any foot
            for j, geom_id in enumerate(foot_geom_ids):
                if contact.geom1 == geom_id or contact.geom2 == geom_id:
                    # Use the magnitude of the normal force (z-component)
                    foot_force_sensor[j] = np.linalg.norm(force[:3])
        
        js.foot_force_sensor = foot_force_sensor
        joint_pub.publish(js)

        # a1 CoM Pose msg
        pose = Pose()
        pose.position.x = pos_quat_trunk[0]
        pose.position.y = pos_quat_trunk[1]
        pose.position.z = pos_quat_trunk[2]
        pose.orientation.w = pos_quat_trunk[3]
        pose.orientation.x = pos_quat_trunk[4]
        pose.orientation.y = pos_quat_trunk[5]
        pose.orientation.z = pos_quat_trunk[6]
        com_pub.publish(pose)

        mj.mj_step(model, data)

        # data.time += 0.001
        # data.ctrl[:] = ctrl
        # mj.mj_forward(model,data)


    # if (data.time>=60):
    #     break;
    
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    # cam.lookat =np.array([ 1.0462306782204185 , 1.743116392639941 , 0.0 ])
    # cam.azimuth = 97.06857368262065 ; cam.elevation = -50.88745261308294 ; cam.distance =  4.28353653567233


    # cam.lookat[0] = data.qpos[0]
    # cam.lookat[1] = data.qpos[1]


    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

node.destroy_node()
rclpy.shutdown()
glfw.terminate()