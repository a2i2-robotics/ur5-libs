import numpy as np
from .robotiq_gripper import RobotiqGripper


class PickAttempt:

    def __init__(self, rtde_c, rtde_r, gripper):
        self.rtde_c = rtde_c
        self.rtde_r = rtde_r
        self.gripper = gripper

    def move_with_increments(self, direction=[0, 0, -1], increment=0.05, max_dist=0.2):
        """Move the arm in the direction by given increments till contact
        :param direction: Direction axis of movement
        :param increment: Increment of movement in each direction
        """
        cur_pos = np.asarray(self.rtde_r.getActualTCPPose())
        i_dir = np.asarray(direction) * increment
        i_dir = np.pad(i_dir, [(0, 3)])

        dist = 0
        new_pos = cur_pos
        while (dist < max_dist):
            t_start = self.rtde_c.initPeriod()

            new_pos += i_dir
            self.rtde_c.moveL(new_pos)
            dist += increment

            self.rtde_c.waitPeriod(t_start)

        return False

    def move_till_contact(self, direction=[0, 0, -0.1, 0, 0, 0], max_dist=0.2):
        """Move the arm in the direction by given increments till contact
        :param direction: Direction axis of movement
        :param max_dist: Max distance to travel if there is no contact
        """
        return self.rtde_c.moveUntilContact(direction, max_dist=max_dist)

    def attempt_single_pick(self, start_pos, open_gripper_at_start=True, max_dist=0.2,
                speed=0.25):
        """
        Moves the tool in z axis downwards for a maximum distance or till contact.
        Try to grab with the gripper

        Returns:
        True if an object was grabbed. False otherwise.
        """
        self.rtde_c.moveL(start_pos, speed=speed)
        if open_gripper_at_start: self.gripper.open()

        # if using the original ur_rtde library use 
        # contact = move_with_increments(max_dist=max_dist)
        contact = self.move_till_contact(max_dist=max_dist)

        if not open_gripper_at_start: self.gripper.open()
        pos, status = self.gripper.close()
        print("Gripper status", status, "position", pos)

        self.rtde_c.moveL(start_pos, speed=speed)

        return status == RobotiqGripper.ObjectStatus.STOPPED_INNER_OBJECT

    def sample_picks(self, center, radius, max_attempts = 100, open_gripper_at_start=True, max_dist=0.5, rng=None):
        """
        Try +max_attempts+ of attempts to pick an object
        within the given bounds

        return the state of each attempt
        """
        _center = np.asarray(center)
        _radius = np.asarray(radius)
        if rng is None: rng = np.random
        offsets = rng.uniform(low=-_radius, high=_radius, size=(max_attempts, 2))
        offsets = np.pad(offsets, [(0, 0), (0, 4)])
        attempt_log = []

        for o in offsets:
            new_cords = _center + o
            success = self.attempt_single_pick(new_cords, open_gripper_at_start, max_dist)
            attempt_log.append([new_cords, success])

            if success: return success, attempt_log

        return success, attempt_log
