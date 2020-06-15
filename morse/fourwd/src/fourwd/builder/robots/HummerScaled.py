from morse.builder import *

class Hummerscaled(GroundRobot):
    """
    A template robot model for HummerScaled, with a motion controller and a pose sensor.
    """
    def __init__(self, name = None, debug = True):

        # HummerScaled.blend is located in the data/robots directory
        GroundRobot.__init__(self, 'fourwd/robots/HummerScaled.blend', name)
        self.properties(classpath = "fourwd.robots.HummerScaled.Hummerscaled",brakes = 0.0, friction = 200.0, force = 0.0,
                        steer = 0.0, init = 0, cid = 0)
