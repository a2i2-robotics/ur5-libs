import numpy as np
from .robotiq_gripper import RobotiqGripper
from ..common import safety


class PickAttempt:

    def __init__(self, rtde_c, rtde_r, gripper, safe=True, reattempts=5, reattempt_radius=0.02):
        self.rtde_c = rtde_c
        self.rtde_r = rtde_r
        self.gripper = gripper
        self.safe = safe
        self.reattempt_num = reattempts
        self.reattempt_radius = reattempt_radius

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

    def move_till_contact(self, direction=[0, 0, -0.1, 0, 0, 0], max_dist=0.2, acceleration=0.25):
        """Move the arm in the direction by given increments till contact
        :param direction: Direction axis of movement
        :param max_dist: Max distance to travel if there is no contact
        """
        if self.safe: safety.assert_acc_limit(acceleration)
        return self.rtde_c.moveUntilContact(direction, max_dist=max_dist, acceleration=acceleration)

    def attempt_single_pick(self, start_pos, open_gripper_at_start=True, max_dist=0.2,
                speed=0.25, acceleration=0.25, gripper_pos=None):
        """
        Moves the tool in z axis downwards for a maximum distance or till contact.
        Try to grab with the gripper

        Returns:
        True if an object was grabbed. False otherwise.
        """
        if self.safe: 
            safety.assert_speed_limit(speed)
            safety.assert_acc_limit(acceleration)

        self.rtde_c.moveL(start_pos, speed=speed, acceleration=acceleration)
        if gripper_pos is not None: 
            actual_q = self.rtde_r.getActualQ()
            actual_q[5] = gripper_pos
            self.rtde_c.moveJ(actual_q)
        if open_gripper_at_start: self.gripper.open()

        # if using the original ur_rtde library use 
        # contact = move_with_increments(max_dist=max_dist)
        contact = self.move_till_contact(max_dist=max_dist, acceleration=acceleration)

        if not open_gripper_at_start: self.gripper.open()
        pos, status = self.gripper.close()
        print("Gripper status", status, "position", pos)

        self.rtde_c.moveL(start_pos, speed=speed, acceleration=acceleration)

        return status == RobotiqGripper.ObjectStatus.STOPPED_INNER_OBJECT, contact

    def sample_picks(self, center, radius, max_attempts = 100, open_gripper_at_start=True, max_dist=0.5, rng=None,
                    speed=0.25, acceleration=0.25, sample_gripper=True, reattempt=True):
        """
        Try +max_attempts+ of attempts to pick an object
        within the given bounds

        return the state of each attempt
        """
        _center = np.asarray(center)
        _radius = np.asarray(radius)
        gripper_pos = None
        if rng is None: rng = np.random
        offsets = rng.uniform(low=-_radius, high=_radius, size=(max_attempts, 2))
        offsets = np.pad(offsets, [(0, 0), (0, 4)])
        attempt_log = []

        for o in offsets:
            new_cords = _center + o
            if sample_gripper: gripper_pos = rng.uniform(low=0.0, high=2.0)
            success, contact = self.attempt_single_pick(new_cords, open_gripper_at_start, max_dist,
                                speed=speed, acceleration=acceleration, gripper_pos=gripper_pos)
            attempt_log.append([new_cords, success])
            print("Contact", contact)

            if reattempt and contact and not success:
                print("Contact detected. Reattempting...")
                success, _log = self.sample_picks(new_cords, self.reattempt_radius, max_attempts=self.reattempt_num,
                                                        max_dist=max_dist, rng=rng, speed=speed, acceleration=acceleration,
                                                        sample_gripper=sample_gripper, reattempt=False)
                attempt_log += _log

            if success: return success, attempt_log

        return success, attempt_log
