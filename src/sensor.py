import rospy
import threading

from depth_jump_sensor.msg import DepthJump

class GapSensor:
    """
    Node determins which of the given depth jumps from the topic depth jumps is a gap.
    A falid gap needs to be wide enough for the robot to fit through.
    """

    def __init__(self):
        rospy.init_node("gap_sensor")

        self.lock = threading.Lock()

    def _init_subscribers(self):
        self._sub_depth_jumps = rospy.Subscriber("depth_jumps", DepthJump, self._receive_depth_jumps)

    def _receive_depth_jumps(self, data):
        """
        Receive depth jump data
        """
        self._process(data)

    def run(self):
        while not rospy.is_shutdown():
            pass

    def _process(self):
        self.lock.acquire()
        try:

        except Exception as ex:
            print(ex.message)

        self.lock.release()


if __name__ == "__main__":
    try:
        gp = GapSensor()
        gp.run()
    except Exception as ex:
        print(ex.message) 