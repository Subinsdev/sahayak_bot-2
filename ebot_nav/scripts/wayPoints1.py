#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose

def movebase_client(g):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = g[0]
    goal.target_pose.pose.position.y = g[1]
    goal.target_pose.pose.orientation.z = g[2]
    goal.target_pose.pose.orientation.w = g[3]

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

# def move_points(points):

if __name__ == '__main__':
    way_points = [(-9.1, -1.2, 0.29, 0.96),
                  (10.7, 10.5,-0.705, 0.71),
                  (12.6, -1.55, 0.707,0.701),
                  (18.2, -1.4, 0.923, 0.4),
                  (-2.0, 4.0, 0.1, 0.1)]
    i = 0
    for goal in way_points:
        try:
            rospy.init_node('movebase_client_py')
            result = movebase_client(goal)
            if result:
                i = i + 1
                rospy.loginfo("Goal execution done! Reached: " + str(i) + " Way-Point, Co-orinates: " + str(goal[0]) + ", "+ str(goal[1]))
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")
