# methods for setting a set of safety parameters for the UR5

MAX_SPEED = 0.25
MAX_ACC = 0.25

def assert_speed_limit(speed, max_limit=MAX_SPEED):
    assert speed <= max_limit, f"Speed {speed} is above the safety limit {max_limit}"

def assert_acc_limit(acc, max_limit=MAX_ACC):
    assert acc <= max_limit, f"Acceleration {acc} is above the safety limit {max_limit}"
