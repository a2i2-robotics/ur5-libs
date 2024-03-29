"""
Common utility/convenience methods used with UR5
"""

import numpy as np
import rtde_control
import rtde_receive
from ..libs.robotiq_gripper import RobotiqGripper
from ..libs.robotiq_gripper import RobotiqGripperDummy


def connect_and_activate_gripper(c_ip, r_ip=None, g_ip=None, g_port=63352, test=False):
    """
    Connect to RTDE control and receiver interfaces
    and active the gripper

    :param c_ip: Controller IP.
    :param r_ip: Receiver IP.
    :param g_ip: Gripper IP.
    :param g_port: Gripper port.

    :returns: Connected RTDE controller, receiver and gripper instances.
    """
    if r_ip is None: r_ip = c_ip
    if g_ip is None: g_ip = c_ip

    try:
        rtde_c = rtde_control.RTDEControlInterface(c_ip)
        rtde_r = rtde_receive.RTDEReceiveInterface(r_ip)

        if not test:
            gripper = RobotiqGripper()
        else:
            gripper = RobotiqGripperDummy()

        print("Connecting to gripper...")
        gripper.connect(g_ip, g_port)

        if not gripper.is_active():
            gripper.activate()

        return rtde_c, rtde_r, gripper
    except RuntimeError as ex:
        print(ex)
        raise ex

def drop_and_return(rtde_c, gripper, center_q, drop_q, speed=1.05, acc=0.25):
    """
    Move the robot arm to a position in joint space,
    open the gripper and return to the base position

    :param rtde_c: RTDE Controller.
    :param gripper: Gripper instance.
    :param center_q: The base joint position.
    :param drop_q: Position in joint space in which robot arm will drop the item.
    :param speed: speed of the movement
    :param acc: acceleration of the arm movement

    :returns: The drop joint position.
    """
    rtde_c.moveJ(drop_q, speed, acc, False)
    gripper.open()
    rtde_c.moveJ(center_q, speed, acc, False)

    return drop_q

def drop_offset_and_return(rtde_c, gripper, center_q, offset=[0,0,0,0,0,0], speed=1.05, acc=0.25):
    """
    Move the robot arm to a joint offset from a base joint position,
    open the gripper and return to the base position

    :param rtde_c: RTDE Controller.
    :param gripper: Gripper instance.
    :param center_q: The base joint position.
    :param offset: Offset from the center_q to which robot arm will drop the item.
    :param speed: speed of the movement
    :param acc: acceleration of the arm movement

    :returns: The drop joint position.
    """
    drop_loc = np.asarray(center_q) + np.asarray(offset)
    return self.drop_and_return(rtde_c, gripper, center_q, drop_loc, speed=speed, acc=acc)

def drop_tcp_and_return(rtde_c, gripper, drop_pos, return_pos, before_pos=None,
                        speed=1.05, acc=0.25, open_gripper=True):
    """
    Move the robot arm to a pose (x,y,z,rx,ry,rz) in base coordinate frame,
    open the gripper and return to the base position

    :param rtde_c: RTDE Controller.
    :param gripper: Gripper instance.
    :param drop_pos: Pose in which robot arm will drop the item.
    :param return_pos: The pose to return to after dropping object.
    :param before_pos: An optional pose to which the arm will travel \
            before moving to the drop pose.
    :param speed: speed of the movement
    :param acc: acceleration of the arm movement

    :returns: The drop pose.
    """
    if before_pos is not None:
        rtde_c.moveL(before_pos, speed, acc, False)
    rtde_c.moveL(drop_pos, speed, acc, False)
    if open_gripper: gripper.open()
    rtde_c.moveL(return_pos, speed, acc, False)

    return drop_pos
