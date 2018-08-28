import sys

import rospy
import diagnostic_updater
import diagnostic_msgs

from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class RCMC(object):
    """ROS Canopen Motor Control
    """

    def __init__(self):
        """
        TODO - DOC
        """
        rospy.init_node('agv_canopen_node')

        # Load variables from the ROS parameter server
        self.node_rate = rospy.get_param('~node_rate', 1)
        self.dig_rate = rospy.get_param('~dig_rate', 30)
        self.path = rospy.get_param('~device/path', '/dev/leaf0')
        self.baudrate = rospy.get_param('~device/baudrate', 1000000)
        self.timeout = rospy.get_param('~device/timeout', 30)
        self.pub_motor_fedback_topic = rospy.get_param('~device/motor_feedback_topic', 'TODO')
        self.pub_velocity = rospy.get_param('~device/velocity_topic', 'TODO')
        self.sub_commands_topic = rospy.get_param('~device/commands_topic', 'TODO')
        self.srv_start_node_topic = rospy.get_param('~device/start_node_topic', 'TODO')

        # Create a diagnostic handler object
        self.updater = diagnostic_updater.Updater()

        # Publisher and subscribers registration
        self.pub_motor_fedback = rospy.Publisher('chatter', String, queue_size=5)
        self.pub_velocity = rospy.Publisher('', Float32, queue_size=5)
        self.sub_commands = rospy.Subscriber(self.sub_commands_topic, JointState, self._sub_commands)
        
        # Services resgistration 
        self.srv_start_node = rospy.Service(self.srv_start_node_topic, Trigger, self._srv_start_node)
        self.srv_stop_node = rospy.Service(self.srv_start_node_topic, Trigger, self._srv_stop_node)
        self.srv_restart_node = rospy.Service(self.srv_start_node_topic, Trigger, self._srv_restart_node)
        self.srv_recover_fault = rospy.Service(self.srv_start_node_topic, Trigger, self._srv_recover_fault)
        
        
        
        
    def __del__(self):
        pass

  # --------------------------------------------------------------------------------------------- #

    def run(self):
        """

        """
        loop_rate = rospy.Rate(self.node_rate)

        try:

            # Configure the diagnostic updater
            self.updater.add(self.bat_dig_ag_topic, self.bms_driver.produce_diagnostics)
            self.updater.setHardwareID('MCAGV1')
            self.updater.period = self.dig_rate

            # Main Loop
            while not rospy.is_shutdown():

                self.updater.update()
                loop_rate.sleep()
        except Exception as ex:
            sys.stdout.write("[ERRO at run] {0}\n".format(ex.__str__()))

  # --------------------------------------------------------------------------------------------- #

    def _srv_start_node(self, req):
        pass

    def _srv_stop_node(self, req):
        pass

    def _srv_restart_node(self, req):
        pass

    def _srv_recover_fault(self, req):
        pass
    
    def _sub_commands(self, req):
        pass

    # --------------------------------------------------------------------------------------------- #


if __name__ == '__main__':
  """
  Responsible to execute the ROS node
  """
  try:
    rmc_node = RCMC()
    rmc_node.run()
  except rospy.ROSInterruptException:
    sys.stdout.write("\n\nSomething awful happened... please refer to the logs.\n\n")
# --------------------------------------------------------------------------------------------- #

