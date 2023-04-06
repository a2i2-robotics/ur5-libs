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

def drop_offset_and_return(rtde_c, gripper, center_q, offset=[0,0,0,0,0,0], speed=1.05, acc=0.25):
    """
    Move the robot arm to a joint offset from a base joint position,
    open the gripper and return to the base position

    :param rtde_c: RTDE Controller.
    :param gripper: Gripper instance.
    :param center_q: The base joint position.
    :param offset: Offset from the center_q to which robot arm will drop the item.

    :returns: The drop joint position.
    """
    #drop_loc = np.asarray(center_q) + np.asarray(offset)
    drop_loc = np.asarray(offset)
    rtde_c.moveJ(drop_loc, speed, acc, False)
    gripper.open()
    rtde_c.moveJ(center_q, speed, acc, False)

    return drop_loc
