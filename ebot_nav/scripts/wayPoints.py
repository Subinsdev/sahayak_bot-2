#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

def movebase_client(g):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = g[0]
    goal.target_pose.pose.position.y = g[1]
    val = quaternion_from_euler(g[3], g[4], g[5])
    goal.target_pose.pose.orientation.x = val[0]
    goal.target_pose.pose.orientation.y = val[1]
    goal.target_pose.pose.orientation.z = val[2]
    goal.target_pose.pose.orientation.w = val[3]

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def run_code(msg):
    print("Started")
    way_points = [(7.66, 2.6, 0, 0, 0, 0), (4.924, -0.8627, 0, 0, 0, -1.56550121307399)]
    for goal in way_points:
        try:
            result = movebase_client(goal)
            if result:
                rospy.loginfo("Goal execution done!")
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")

if __name__ == '__main__':
    #way_points = [(-9.1, -1.2), (10.7, 10.5), (12.6, -1.9), (18.2, -1.4), (-2.0, 4.0)]
    #way_points = [(10.7, 10.5, -1.0), (12.6, -1.9, -1.0), (18.2, -1.4, -1.0), (-2.0, 4.0, -1.0)]
    #(12.8655481339, 1.19009208679)
    rospy.init_node('movebase_client_py')
    print("Waypoints stated.......................................................")
    print(".........................................................................")
    print(".........................................................................")
    print(".........................................................................")
    print(".........................................................................")
    print(".........................................................................")
    print(".........................................................................")
    print(".........................................................................")
    print(".........................................................................")
    sub1 = rospy.Subscriber("/status", Pose, queue_size=10, callback=run_code)
    rospy.spin()
