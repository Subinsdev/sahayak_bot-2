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
    # val = quaternion_from_euler(0, 0, g[3])
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = g[2]
    goal.target_pose.pose.orientation.w = g[3]

    client.send_goal(goal)
    wait = client.wait_for_result()
    print (g);
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def run_code(msg):
    print("Started")
    # way_points = [(7.66, 2.6, 0, 0, 0, 0), (4.924, -0.8627, 0, 0, 0, -1.56550121307399)]
    way_points = [
                #Object 1
                (13.035, 0.99, -0.702, 0.711), #Pantry Intermediate
                (13.035, -0.1, -0.702, 0.711), #Pantry Intermediate
                (13.035, -0.1, -0.182, 0.98), #Pantry Intermediate
                (14.68, -0.511, -0.718, 0.695), #Pantry Pickup Table 1
                (14.68, -0.511, 1, 0.0008), #Pantry Pickup Table 1
                (11.233, -1.222, 1, 0.0008), #Pantry Pickup Table 2 Position 1
                (11.233, -1.222, -0.709, -0.70), #Pantry Pickup Table 2 Position 1
                (11.233, -1.222, 0.3824, 0.92395), #Pantry Pickup Table 2 Position 1 Orientation
                (13.2159, -0.604, -0.7577, -0.650), #Pantry Out OIntermediate
                (13.02, 0.647, -0.719, -0.694), #Pantry Out OIntermediate

                ( 8.638, 1.148, 0.7063, 0.7078), #Meeting Intermediate CV
                ( 8.62, 2.132, 0.7063, 0.7078), #Meeting Intermediate 2 CV
                ( 6.9, 2.6, 0.00, 0.010), #Meeting DropBox
                # #Object 2
                ( 7.999, 2.4, 0.0, 0.007), #Meeting Pickup
                ( 8.5, 2.175, -0.6564, 0.7543), #Meeting Intermediate out CV
                ( 8.638, 1.148, -0.6564, 0.7543), #Meeting Intermediate CV


                (11.400, 10.010, -0.018, -1.00), #Reaserch DropBox along length
                (14.2346, 10.1097, -0.011123, -0.9936), #Reaserch DropBox Intermediatie

                #Object3
                (26.165, -2.714, -0.891, 0.454),  #Store Pickup 1
                (25.8179, -3.2344, -0.8869, 0.462), #Store Pickup 2
                (25.8179, -3.2344, 0.894, -0.448), #Store Pickup 2 out
                (15.5, 0.9, 1, 0.0), #Store Pickup 2 out
                ( 5.156, 0.861, -0.706, 0.7082), #Conference Intermediate CV
                ( 5.070, -0.771, -0.9126, 0.4087), #Conference DropBox CV
                ( 5.070, -0.771, -0.7794, -0.625), #Conference Intermediate out CV
                ( 5.170, 0.2, -0.7794, -0.625), #Conference Intermediate out CV
                #
                (0,0,0,1)]                   #Start


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
