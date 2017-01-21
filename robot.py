class Sensor(object):
    def __init__(self):
        pass
    def sense(self):
        raise NotImplementedError()

class GPS(sensor):
    def __init__(self):
        pass

class IMU(sensor):
    def __init__(self):
        pass

class Encoder(sensor):
    def __init__(self):
        pass

class Robot(object):
    def __init__(self):
        pass
    def move(self, cmd):
        pass
    def sense(self, env):
        # Simulated GPS, IMU, and Odometry
        pass
