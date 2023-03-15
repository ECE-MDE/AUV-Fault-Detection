import rospy
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
    rospy.init_node('listener', anonymous=True)

    sim_state_print_rate = rospy.Rate(0.5)
    setpt_pub_rate = rospy.Rate(1)

    def print_sim_state(msg: VehicleStateMsg):
        print(f"{msg}\r")
        sim_state_print_rate.sleep()


    # Create a listener for the sim state topic and print the sim state to the terminal every time it's published
    rospy.Subscriber("/sim/state", VehicleStateMsg, print_sim_state, queue_size=1)

    # Print a list of all currently published topics
    for name, type in rospy.get_published_topics():
        print(f"{name} {type}")

    # Create a publisher for RPM setpoint topic that publishes the `Float64SetpointMsg` topic
    pub_rpm_setpt = rospy.Publisher("setpoint/rpm", Float64SetpointMsg)


    while not rospy.is_shutdown():
        pub_rpm_setpt.publish(Float64SetpointMsg(True, 10))
        setpt_pub_rate.sleep()
        ...

if __name__ == '__main__':
    main()