import math
import rospy
from avl_fault_detection import logger
from avl_msgs.msg import VehicleStateMsg, Float64SetpointMsg
"""
surge velocity
heave velocity
depth rate

roll angle
pitch angle
yaw angle

roll rate
pitch rate
yaw rate
"""

def main():
    # Create a node
    rospy.init_node('controller', anonymous=True)

    sim_state_print_rate = rospy.Rate(1)
    setpt_pub_rate = rospy.Rate(1)

    # Create a subscriber for the sim state topic and print the sim state to the terminal every 1 second
    def print_sim_state(msg: VehicleStateMsg):
        print(f"{msg}\r")
        sim_state_print_rate.sleep()

    # Create RPM and pitch setpoint publishes to get the AUV moving
    pub_rpm_setpt = rospy.Publisher("setpoint/rpm", Float64SetpointMsg)
    pub_pit_setpt = rospy.Publisher("setpoint/pitch", Float64SetpointMsg)

    rospy.Subscriber("/sim/state", VehicleStateMsg, print_sim_state, queue_size=1)

    log = logger.AvlLogger('controller')
    log.write_header(["fault_type"], ["none"])

    # Print a list of all currently published topics
    for name, topic_type in rospy.get_published_topics():
        print(f"{name} {topic_type}")

    while not rospy.is_shutdown():
        pub_rpm_setpt.publish(Float64SetpointMsg(True, 10))
        pub_pit_setpt.publish(Float64SetpointMsg(True, math.radians(-45)))
        if rospy.Time.now() < 15:
            log.write_data(["none"])
        else:
            log.write_data(["depth_sensor_zeroed"])
        setpt_pub_rate.sleep()
        ...
    
    log.close()

if __name__ == '__main__':
    main()