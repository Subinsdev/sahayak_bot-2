#! /usr/bin/env python

################################################################################
# We have used the find 2-D object package to the task, we noticed in the      #
# last task that we needed better object identifing algorithms, we spent       #
# a lot of time in trying other methods, but finally we were able to improve   #
# the identification by downloading extra algos for find_object_2d and we      #
# were succefull in doing so, we have also created an algo where we can        #
# determine the best path to take when given were the ebot will visit in       #
# the smallest time possible by giving us the rooms that need to be entered    #
# and in which order it can also it will tell which order the objects are      #
# to be picked, drive link that will be given which will take you to the       #
# sessions used for the task. we have taken images  of each object in          #
# different loacations so  that detection is robost,  task6.launch has been    #
# modified so  that the find 2-D pkg also starts from it that launch file      #
# has also been given in drive. One unique thing we did in the past tasks was  #
# adding objects to rviz so that a proper plan can be planned without hitting  #
# objects. The control.py is the main code which room to visit and the which   #
# order to pickup objects in the shortest time order. graph_path.py is the one #
# that find the path and give to control.py and control.py will analyse.       #
# I hope the method that we have used to approch the various themes   	       #
# is upto your mark. Thank you for the opportunity to participate              #
################################################################################

#drive link:https://drive.google.com/drive/folders/1cmiCYpuLrB6xugoWHR5D4ElcX10z_b1E?usp=sharing

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import time
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_ros_planning_interface import _moveit_planning_scene_interface
from geometry_msgs.msg import PoseStamped, Pose, Point
from object_msgs.msg import ObjectPose
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
import pyassimp

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from find_object_2d.msg import ObjectsStamped
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
# import ur5_moveit

names = ["Wheels", "EYFI", "FPGA", "Battery", "Glue", "Coke", "Adhesive", "Glass"]

flag_object = np.zeros(8)


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self.gripper = moveit_commander.MoveGroupCommander("hand")
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        # rospy.loginfo(
        #     '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        # rospy.loginfo(
        #     '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        # rospy.loginfo(
        #     '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        # rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

        scene = moveit_commander.PlanningSceneInterface()
        # ROS answers suggested adding a sleep here to allow the PlanningSceneInterface
        # to initialize properly; we have noticed that collision objects aren't
        # always received without it.

        self._psi = _moveit_planning_scene_interface.PlanningSceneInterface()
        self._pub_co = rospy.Publisher('/collision_object', CollisionObject, queue_size=100)
        self._pub_aco = rospy.Publisher('/attached_collision_object', AttachedCollisionObject, queue_size=100)

        rospy.sleep(2.0)

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        angles = euler_from_quaternion([pose_values.orientation.x, pose_values.orientation.y,pose_values.orientation.z,pose_values.orientation.w])
        # print("CURRENT ANGLE")
        # rospy.loginfo(angles)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)
        #
        # if (flag_plan == True):
        #     rospy.loginfo(
        #         '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        # else:
        #     rospy.logerr(
        #         '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def add_box(self, name, pose, size = (0.1, 0.1, 0.1)):
        """
        Add a box to the planning scene
        """
        self._pub_co.publish(self.make_box(name, pose, size))

    def make_box(self, name, pose, size):
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size)
        co.primitives = [box]
        co.primitive_poses = [pose.pose]
        return co

    def add_mesh(self, name, pose, filename, size = (1, 1, 1)):
        self._pub_co.publish(self.__make_mesh(name, pose, filename, size))

    def __make_mesh(self, name, pose, filename, scale = (1, 1, 1)):
        co = CollisionObject()
        scene = pyassimp.load(filename)
        if not scene.meshes:
            raise MoveItCommanderException("There are no meshes in the file")
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header

        mesh = Mesh()
        for face in scene.meshes[0].faces:
            triangle = MeshTriangle()
            if len(face) == 3:
                triangle.vertex_indices = [face[0], face[1], face[2]]
            mesh.triangles.append(triangle)
        for vertex in scene.meshes[0].vertices:
            point = Point()
            point.x = vertex[0]*scale[0]
            point.y = vertex[1]*scale[1]
            point.z = vertex[2]*scale[2]
            mesh.vertices.append(point)
        co.meshes = [mesh]
        co.mesh_poses = [pose.pose]
        pyassimp.release(scene)
        return co

    def go_to_joint(self, joint_location):
        self._group.set_joint_value_target(joint_location) #[1.1, -0.9, 1.28, -1.92, -1.54, -0.21]
        #self._group.plan()
        flag_plan = self._group.go(wait=True)

    def closeGripper(self, temp):
        #self.gripper.set_named_target("close")
        #temp = float(input("Enter how much to close: "))
        self.gripper.set_joint_value_target({'gripper_finger1_joint': temp})
        flag_close = self.gripper.go(wait=True)
        # if flag_close:
        #     rospy.loginfo(
        #         '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        # else:
        #     rospy.logerr(
        #         '\033[94m' + ">>> go_to_pose() Failed. Cannot close" + '\033[0m')

        return flag_close

    def go_to_joint(self, joint_location):
        self._group.set_joint_value_target(joint_location) #[1.1, -0.9, 1.28, -1.92, -1.54, -0.21]
        flag_plan = self._group.go(wait=True)

    def openGripper(self):
        self.gripper.set_named_target("open")
        flag_open = self.gripper.go(wait=True)
        # if flag_open:
        #     rospy.loginfo(
        #         '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        # else:
        #     rospy.logerr(
        #         '\033[94m' + ">>> go_to_pose() Failed. Cannot close" + '\033[0m')

        return flag_open

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        # rospy.loginfo(
        #     '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

    def remove_world_obj(self, name = None):
        co =  CollisionObject()
        co.operation =  CollisionObject.REMOVE
        if name != None:
            co.id = name
        self._pub_co.publish(co)

def movebase_client(g):
    # print(g)
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
    # print("way Point:    ",g)
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        # rospy.logerr("Action server not available!")
        # rospy.signal_shutdown("Action server not available!")
        pass
    else:
        return client.get_result()

def findObjects():
    listener = tf.TransformListener()
    return_object_id = [-1]*8
    return_object_tranform = [0]*8
    rate = rospy.Rate(20.0)
    obj_name = '/object_'
    start_time = time.time()
    end_time = time.time()
    object_ids = [[59, 71, 78, 89, 97, 99, 106, 108, 120, 127, 128, 133, 137, 139],                 #Wheels
                    [57, 74, 76, 82, 104, 107, 121, 129, 135, 136,143],                             #EYFI
                    [55, 70, 73, 75, 83, 88, 105, 118, 124, 130,141],                               #FPGA
                    [41, 56, 61, 63, 80, 98, 101, 102, 103, 113, 114, 117, 123, 125, 126,140,142],  #Battery
                    [43, 58, 64, 85, 93, 96, 110, 115, 116, 119, 122, 132, 134, 138,144],           #Glue
                    [44, 68, 69, 84, 87, 90],                                                       #Coke
                    [42, 62, 72, 81, 86, 92, 95, 100, 109, 111, 112, 131],                          #Adhesive
                [45, 48, 49, 50, 51, 52, 53, 66, 67, 91]]                                           #Glass

    while end_time-start_time < 2:
        for i in range(len(object_ids)):
            for j in range(len(object_ids[i])):
                obj_id = obj_name + str(object_ids[i][j])
                try:
                    (tran1, _) = listener.lookupTransform('/ebot_base', obj_id, rospy.Time(0))
                    (tran2, _) = listener.lookupTransform('/base_link', obj_id, rospy.Time(0))
                    return_object_id[i] = object_ids[i][j]
                    return_object_tranform[i] = [tran1, tran2]
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
        rate.sleep()
        end_time = time.time()
        # print("Time completed: ", end_time-start_time)
    # x = input("Wait:   ")
    # for j in range(8):
    #     if return_object_id[j] != -1:
    #         print(str(names[j]) + " Identified")
    return return_object_id, return_object_tranform


def addObjectBox(name, position, size, tolerance, angle):
    obj = geometry_msgs.msg.PoseStamped()
    obj.header.frame_id = "/ebot_base"
    obj.pose.position.x = position[0]+tolerance[0]
    obj.pose.position.y = position[1]+tolerance[1]
    obj.pose.position.z = position[2]+tolerance[2]
    angles = quaternion_from_euler(angle[0], angle[1], angle[2])
    obj.pose.orientation.x = angles[0]
    obj.pose.orientation.y = angles[1]
    obj.pose.orientation.z = angles[2]
    obj.pose.orientation.w = angles[3]

    ur5.add_box(name, obj, size)

def get_publish_obj(name, position):
    obj = ObjectPose()
    obj.name = name
    obj.pose.header.frame_id = "/base_link"
    obj.pose.pose.position.x = position[0]
    obj.pose.pose.position.y = position[1]
    obj.pose.pose.position.z = position[2]
    angles = quaternion_from_euler(0, 0, 0)
    obj.pose.pose.orientation.x = angles[0]
    obj.pose.pose.orientation.y = angles[0]
    obj.pose.pose.orientation.z = angles[0]
    obj.pose.pose.orientation.w = angles[0]
    detection_pub.publish(obj)

def add_dected_objects_mesh_in_rviz(ur5, object_ids, object_tranforms):
    obj = geometry_msgs.msg.PoseStamped()
    angles = quaternion_from_euler(0, 0, 0)
    obj.header.frame_id = "/ebot_base"
    obj.pose.orientation.x = angles[0]
    obj.pose.orientation.y = angles[1]
    obj.pose.orientation.z = angles[2]
    obj.pose.orientation.w = angles[3]
    # paths = ["/home/chandravaran/catkin_ws/src/sahayak_bot/ebot_gazebo/models/robot_wheels/meshes/robot_wheels.dae",
    # "/home/chandravaran/catkin_ws/src/sahayak_bot/ebot_gazebo/models/eYIFI/meshes/eyifi.dae",
    # "/home/chandravaran/catkin_ws/src/sahayak_bot/ebot_gazebo/models/biscuits/meshes/biscuits.dae",
    # "/home/chandravaran/catkin_ws/src/sahayak_bot/ebot_gazebo/models/soap2/meshes/soap2.dae",
    # "/home/chandravaran/catkin_ws/src/sahayak_bot/ebot_gazebo/models/glue/meshes/glue.dae",
    # "/home/chandravaran/catkin_ws/src/sahayak_bot/ebot_gazebo/models/coke_can/meshes/coke_can.dae",
    # "/home/chandravaran/catkin_ws/src/sahayak_bot/ebot_gazebo/models/adhesive/meshes/adhesive.dae",
    # "/home/chandravaran/catkin_ws/src/sahayak_bot/ebot_gazebo/models/water_glass/meshes/glass.dae"]

    paths = ["/home/shadowfox/ws_eyantra/src/sahayak_bot/ebot_gazebo/models/robot_wheels/meshes/robot_wheels.dae",
    "/home/shadowfox/ws_eyantra/src/sahayak_bot/ebot_gazebo/models/eYIFI/meshes/eyifi.dae",
    "/home/shadowfox/ws_eyantra/src/sahayak_bot/ebot_gazebo/models/biscuits/meshes/biscuits.dae",
    "/home/shadowfox/ws_eyantra/src/sahayak_bot/ebot_gazebo/models/soap2/meshes/soap2.dae",
    "/home/shadowfox/ws_eyantra/src/sahayak_bot/ebot_gazebo/models/glue/meshes/glue.dae",
    "/home/shadowfox/ws_eyantra/src/sahayak_bot/ebot_gazebo/models/coke_can/meshes/coke_can.dae",
    "/home/shadowfox/ws_eyantra/src/sahayak_bot/ebot_gazebo/models/adhesive/meshes/adhesive.dae",
    "/home/shadowfox/ws_eyantra/src/sahayak_bot/ebot_gazebo/models/water_glass/meshes/glass.dae"]

    # paths = ["/home/ubuntu1804/eyantra_ws/src/sahayak_bot/ebot_gazebo/models/robot_wheels/meshes/robot_wheels.dae",
    # "/home/ubuntu1804/eyantra_ws/src/sahayak_bot/ebot_gazebo/models/eYIFI/meshes/eyifi.dae",
    # "/home/ubuntu1804/eyantra_ws/src/sahayak_bot/ebot_gazebo/models/biscuits/meshes/biscuits.dae",
    # "/home/ubuntu1804/eyantra_ws/src/sahayak_bot/ebot_gazebo/models/soap2/meshes/soap2.dae",
    # "/home/ubuntu1804/eyantra_ws/src/sahayak_bot/ebot_gazebo/models/glue/meshes/glue.dae",
    # "/home/ubuntu1804/eyantra_ws/src/sahayak_bot/ebot_gazebo/models/coke_can/meshes/coke_can.dae",
    # "/home/ubuntu1804/eyantra_ws/src/sahayak_bot/ebot_gazebo/models/adhesive/meshes/adhesive.dae",
    # "/home/ubuntu1804/eyantra_ws/src/sahayak_bot/ebot_gazebo/models/water_glass/meshes/glass.dae"]


    scale = [(1, 1, 1), (0.1, 0.1, 0.1), (1, 1, 1), (1, 1, 1), (0.1, 0.1, 0.1), (0.1, 0.1, 0.1), (1, 1, 1), (1, 1, 1)]
    for i in range(len(object_ids)):
        if object_ids[i]!=-1:
            trans = object_tranforms[i][0]
            obj.pose.position.x = trans[0]
            obj.pose.position.y = trans[1]
            obj.pose.position.z = trans[2]
            ur5.add_mesh(names[i], obj, paths[i], scale[i])

def remove_detected_objects_mesh_in_rviz(object_ids):
    for i in range(len(object_ids)):
        if object_ids[i]!=-1:
            ur5.remove_world_obj(names[i])

def showDetectedObjects(msg):
    data = msg.objects.data
    global flag_object
    detected_objects_image = image.copy()
    object_ids = [[59, 71, 78, 89, 97, 99, 106, 108, 120, 127, 128, 133, 137, 139],                 #Wheels
                    [57, 74, 76, 82, 104, 107, 121, 129, 135, 136,143,146],                             #EYFI
                    [55, 70, 73, 75, 83, 88, 105, 118, 124, 130,141,147],                               #FPGA
                    [41, 56, 61, 63, 80, 98, 101, 102, 103, 113, 114, 117, 123, 125, 126,140,142,145],  #Battery
                    [43, 58, 64, 85, 93, 96, 110, 115, 116, 119, 122, 132, 134, 138,144],           #Glue
                    [44, 68, 69, 84, 87, 90],                                                       #Coke
                    [42, 62, 72, 81, 86, 92, 95, 100, 109, 111, 112, 131],                          #Adhesive
                [45, 48, 49, 50, 51, 52, 53, 66, 67, 91]]                                           #Glass
    detected = {}
    for i in range(0, len(data), 12):
        idx = int(data[i])
        w = data[i+1]
        h = data[i+2]

        a = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype='float32')
        h = np.array([[data[i+3], data[i+6], data[i+9]], [data[i+4], data[i+7], data[i+10]], [data[i+5], data[i+8], data[i+11]]], dtype='float32')
        a = np.array([a])

        pointsOut = cv2.perspectiveTransform(a, h)
        detected[idx] = np.int32(pointsOut)

    for i in range(len(object_ids)):
        for idx in object_ids[i]:
            if idx in detected:
                if(flag_object[i] == 0):
                    print(names[i] + ' Detected')
                    flag_object[i] = 1
                detected_objects_image = cv2.polylines(detected_objects_image, [detected[idx]], True, (255, 0, 0), 3)
                [x, y] = detected[idx][0][0]
                org = (x, y+100)
                fontScale = 1
                color = (255, 255, 0)
                thickness = 2
                detected_objects_image = cv2.putText(detected_objects_image, names[i], org, cv2.FONT_HERSHEY_SIMPLEX ,  fontScale, color, thickness, cv2.LINE_AA)
                break

    cv2.imshow("Detected objects", detected_objects_image)
    cv2.waitKey(1)


def storeImage(msg):
    global image
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


def main():
    global ur5
    global flag_object
    ur5 = Ur5Moveit()

    sub1 = rospy.Subscriber('/objectsStamped', ObjectsStamped, callback=showDetectedObjects, queue_size=10)
    sub2 = rospy.Subscriber('/camera/color/image_raw2', Image, callback=storeImage, queue_size=10)

    # print("Waiting for image")
    # while True:
    #     try:
    #         image
    #         break
    #     except:
    #         continue
    print("Launched Detectetion Object Window")
    flag_object = np.ones(8)



    way_points = [
                (3.6, 0.85,0,1), #0  enter the hallway point

                ######### Pantry Room ########
                (13.035, 0.99, -0.702, 0.711),      #1  Pantry Intermediate outside door
                (13.035, -0.1, -0.702, 0.711),      #2  Pantry Intermediate inside door
                (13.035, -0.1, -0.182, 0.98),       #3  Pantry Intermediate inside door orientation
                (14.695, -0.75, -0.718, 0.695),     #4  Pantry Pickup Table 1
                (14.615, -0.62, 1, 0.0008),         #5  Pantry Pickup Table 1 - Orientation to Table2
                (11.15, -1.32, -0.709, -0.70),      #6  Pantry Pickup Table 2 Position 1
                (11.15, -1.32, 0.3824, 0.92395),    #7  Pantry Pickup Table 2 Position 1 Orientation
                (13.2159, -0.604, -0.7577, -0.650), #8  Pantry Out OIntermediate
                (13.0, 0.8, -0.719, -0.694),        #9  Pantry Out OIntermediate

                ######### Meeting Room ########
                ( 8.64, 1.148, 0.7063, 0.7078),     #10  Meeting Intermediate
                ( 8.64, 2.4, 0.5646, 0.8253),     #10  Meeting Intermediate
                # ( 8.64, 2.4, 1.0, 0.0003),          #11  Meeting Intermediate
                ( 6.9, 2.6, 0.0, 0.010),            #12  Meeting DropBox
                ( 7.6, 2.4, 0.0, 0.007),            #13  Meeting Pickup
                ( 8.55, 2.175, -0.7068, 0.7073),     #14  Meeting Intermediate out
                ( 8.5, 1.148, -0.7068, 0.7073),     #15  Meeting Intermediate outside Door

                ####### Meeting to Reaserch DropBox #######
                (11.200, 10.010, -0.018, -1.00),    #16  Reaserch DropBox along length
                (14.6, 10.1097, -0.011123, -0.9936),#17  Reaserch DropBox Intermediatie to Store Room
                (15.0, 3.9, -0.623, 0.781),         #18  Research lab to store room Intermediate

                #######   Store Room   #######
                (26.065, -2.714, -0.891, 0.454),    #19  Store Pickup 1
                (26.065, -2.714, -0.994, 0.0),      #20  Store Pickup 1 Orientation to Exit
                (25.8179, -3.3344, -0.871, 0.490),  #21  Store Pickup 2
                (25.8179, -3.3344, 0.894, -0.448),  #22  Store Pickup 2 Orientation to Exit
                (16.7, 1.0, 1, 0.0),                #23  Store Pick,up 2 out

                ####### Conference Room ######
                ( 5.156, 0.861, -0.706, 0.7082),    #24  Conference Intermediate outside Door when enetering
                ( 5.070, -0.7, -0.9126, 0.4087),    #25  Conference DropBox
                ( 5.070, -0.7, -0.7583, -0.6518),   #26  Conference Intermediate turn Towards Dooe
                ( 5.18, 0.1, -0.7583, -0.6518),     #27  Conference Intermediate inside Door
                ( 5.25, 0.65, -0.7583, -0.6518),    #28  Conference Intermediate outside Door

                ## Start to store Extra intermediate ##
                (16.7,1.0,0,1),                     #29

                (15.0, 3.9, -0.666, -0.74542),      #30  Research lab to store room Intermediate
                (14.8, 10.237 , -0.9999 , 0),       #31  Research lab Turning towards the Dropbox
                (11.2, 10.010, -0.91537, 0.402),    #32  Research Going to DropBox

                #### The out point when we dont go to drop box #######
                ( 8.6, 2.175, -0.7068, 0.7073),      #33   Meeting Intermediate out CV

                (9.20,4.89,0.999,0.0359),            # 34 meeting room enter first intermediate
                (5.6, 4.95 , 0.999, -0.005),          # 35 meeting pickup
                (4.3376, 4.80 , 0.894,-0.4463),      # 36 meeting room intermediate from pickup
                (4.115,2.6211,0.034, -0.9994),      # 37intermediate facing drop box
                ###### Back to Start #########
                (0,0,-0.719, -0.6940)              # 38 Start Point
                ]

    lst_joint_angles_1 = [-0.4, 0.74, 0.31, -0.02, -0.96, -0.02]
    lst_joint_angles_2 = [-0.5, 0.74, 0.31, -0.02, -0.96, -0.02]


    #going to the crouch position
    # while not rospy.is_shutdown():
    f = ur5.go_to_joint(lst_joint_angles_1)
        # if f:
        #     # obj = Pose()
        #     # pub1.publish(1obj)
        #     break
        # rospy.sleep(2)
    # movebase_client((0,0,0,0))
    print("Started Run!")
    movebase_client(way_points[0])
    #
    # movebase_client(way_points[29])
    movebase_client(way_points[19])

    print("Store Room Reached")
    flag_object = np.zeros(8)

    #################################### Battery ##################################
    ############################ Looking Left Side  ############################
    states=[[0, -0.37, -0.785, -1, -0.52, 1.57], [0.56, -0.37, -0.785, -1, -0.65, 1.57]]
    for state in states:
        ur5.go_to_joint(state)
        # input("wait:   ")
        object_ids, object_tranforms  = findObjects()
        if object_ids[3]!=-1:
            x, y, z, rot, close_val = -0.07, -0.18, 0.20, -0.3, 0.45
            if object_ids[3]==117:
                x, y, z, rot = 0.005, -0.18, 0.19, 0
            elif object_ids[3]==125:
                x, y, z, rot, close_val = 0.17, -0.05, 0.21, 1.2, 0.49
            elif object_ids[3]==126:
                x, y, z, rot, close_val = -0.005, -0.18, 0.21, 0, 0.43
            # input("wait:   ")
            ur5_pose_1 = geometry_msgs.msg.Pose()
            trans = object_tranforms[3][0]
            ur5_pose_1.position.x = trans[0]+x
            ur5_pose_1.position.y = trans[1]+y
            ur5_pose_1.position.z = trans[2]+z
            angles = quaternion_from_euler(3.8, 0, -3.14+rot)
            ur5_pose_1.orientation.x = angles[0]
            ur5_pose_1.orientation.y = angles[1]
            ur5_pose_1.orientation.z = angles[2]
            ur5_pose_1.orientation.w = angles[3]
            ur5.go_to_pose(ur5_pose_1)

            ur5_pose_1.position.z = trans[2]+0.155
            ur5.go_to_pose(ur5_pose_1)
            ur5.closeGripper(close_val)

            ur5.go_to_joint([0.5, -0.37, -0.785, -1, -0.65, 1.57])
            # input("wait:   ")
            ur5.go_to_joint([-0.5, -0.37, -0.785, -1, -0.55, 1.57])
            # input("wait:   ")
            print(str(names[3]) + " Picked")
            ur5.go_to_joint(lst_joint_angles_2)
            movebase_client(way_points[20])
            break

    if object_ids[3]==-1:
        ur5.go_to_joint(lst_joint_angles_1)
        movebase_client(way_points[21])

        #################RIGHT SIDE##################
        states=[[0, -0.37, -0.885, -1, -0.7, 1.57]]
        for state in states:
            ur5.go_to_joint(state)
            object_ids, object_tranforms  = findObjects()

            if object_ids[3]!=-1:
                x, y, z, close_val = -0.005, -0.18, 0.21, 0.43
                if object_ids[3]==103:
                    x, y, z = -0.015, -0.18, 0.21
                elif object_ids[3]==114:
                    x, y, z, close_val = 0.005, -0.18, 0.21, 0.3

                ur5_pose_1 = geometry_msgs.msg.Pose()
                trans = object_tranforms[3][0]
                ur5_pose_1.position.x = trans[0]+x
                ur5_pose_1.position.y = trans[1]+y
                ur5_pose_1.position.z = trans[2]+z
                angles = quaternion_from_euler(3.8, 0, -3.14)
                ur5_pose_1.orientation.x = angles[0]
                ur5_pose_1.orientation.y = angles[1]
                ur5_pose_1.orientation.z = angles[2]
                ur5_pose_1.orientation.w = angles[3]
                ur5.go_to_pose(ur5_pose_1)

                ur5_pose_1.position.z = trans[2]+0.155
                ur5.go_to_pose(ur5_pose_1)
                ur5.closeGripper(close_val)
                # ur5.go_to_joint([0.5, -0.37, -0.785, -1, -0.65, 1.57])
                ur5.go_to_joint([-0.5, -0.37, -0.785, -1, -0.65, 1.57])
                print(str(names[3])+ " Picked")

                ur5.go_to_joint(lst_joint_angles_2)
                movebase_client(way_points[22])
                break

    if(object_ids[3]==-1):
        ur5.go_to_joint(lst_joint_angles_1)
        movebase_client(way_points[22])
    flag_object = np.ones(8)

    for i in range(30,33):
        movebase_client(way_points[i])

    print("Research Lab Reached")

    state=[-0.2, 0.23, -1.12, 0, 1.36, 0]
    ur5.go_to_joint(state)
    state=[0.8, 0.4, -1.12, -1, 0.2, 1.57]
    ur5.go_to_joint(state)

    ##################  Dropping the Battery  #################
    ur5.openGripper()
    print(str(names[3]) + " Dropped in Dropbox 3")
    state=[-0.2, 0.23, -1.12, 0, 1.36, 0]
    ur5.go_to_joint(state)
    ur5.go_to_joint(lst_joint_angles_1)



    ############################### Pantry room #############################
    # Moving to Table 1
    for i in range(1,5):
        movebase_client(way_points[i])
        if i == 3:
            print("Pantry Reached")
            flag_object = np.zeros(8)

    ############################ Finding Coke ################################

    states=[[-0.05, -0.37, -0.785, -1, -0.8, 1.57]]
    for i in range(len(states)):
        ur5.go_to_joint(states[i])
        # object_ids, object_tranforms  = findObjects()
        z=0

    ############# This goes to the next table in the pantry ##########
    ur5.go_to_joint(lst_joint_angles_1)

    if (flag_object[5] == 0) or (flag_object[7] == 0):
        ur5.go_to_joint(lst_joint_angles_1)
        movebase_client(way_points[5])
        movebase_client(way_points[6])
        states=[[-0.07, -0.27, -0.785, -1, -0.8, 1.57]]
        for i in range(len(states)):
            ur5.go_to_joint(states[i])
            # object_ids, object_tranforms  = findObjects()
            z=0
            if flag_object[5]!=0:
                ur5.go_to_joint(lst_joint_angles_1)
                movebase_client(way_points[7])
                break

    if (flag_object[5] == 0) or (flag_object[7] == 0):
        ur5.go_to_joint(lst_joint_angles_1)
        movebase_client(way_points[7])


    for i in range(8,10):
        movebase_client(way_points[i])

    flag_object = np.ones(8)
    movebase_client(way_points[10])
    movebase_client(way_points[11])

    print("Meeting Room Reached")
    flag_object = np.zeros(8)

    # movebase_client(way_points[34])
    movebase_client(way_points[35])

    #Picking up glue
    states=[[0, -0.47, -0.785, -1, -0.72, 1.57]]

    for i in range(len(states)):
        ur5.go_to_joint(states[i])
        object_ids, object_tranforms  = findObjects()
        # print(object_ids)
        # print(object_tranforms)
        # print("Adding deteced objects in rviz")
        # add_dected_objects_mesh_in_rviz(ur5, object_ids, object_tranforms)
        # print("Done")
        z=0
        if object_ids[4]!=-1:
            # while not rospy.is_shutdown():
            # print("Found object_", object_ids[4])
            ur5_pose_1 = geometry_msgs.msg.Pose()
            trans = object_tranforms[4][0]
            x, y, z = 0.007, - 0.3, + 0.2
            # input('wait: ')
            # x = float(input("Enter x: "))
            # y = float(input("Enter y: "))
            # z = float(input("Enter z: "))
            ur5_pose_1.position.x = trans[0]+x
            ur5_pose_1.position.y = trans[1]+y
            ur5_pose_1.position.z = trans[2]+z
            angles = quaternion_from_euler(3.68, 0, -3.14)
            ur5_pose_1.orientation.x = angles[0]
            ur5_pose_1.orientation.y = angles[1]
            ur5_pose_1.orientation.z = angles[2]
            ur5_pose_1.orientation.w = angles[3]
            ur5.go_to_pose(ur5_pose_1)
            # flag = int(input("Close the gripper: "))
            # if flag==1:
            #     break
            y = -0.185
            ur5_pose_1.position.y = trans[1]+y
            ur5_pose_1.position.z = trans[2]+z-0.075
            ur5.go_to_pose(ur5_pose_1)

            ur5.closeGripper(0.32)
            ur5.go_to_joint(states[i])
            print(str(names[4]) + " Picked")
            # remove_detected_objects_mesh_in_rviz(object_ids)
            break

    ur5.go_to_joint(lst_joint_angles_2)



    # movebase_client(way_points[36])
    movebase_client(way_points[12])

    state=[-0.2, 0.23, -1.12, 0, 1.36, 0]
    ur5.go_to_joint(state)

    state=[1.13, 0.4, -1.12, 0, 1.36, 0]
    ur5.go_to_joint(state)

    #dropping the glue can
    # print("Opening gripper")
    ur5.openGripper()

    print(str(names[4]) + " Dropped in Dropbox 2")

    state=[-0.2, 0.23, -1.12, 0, 1.36, 0]
    ur5.go_to_joint(state)

    ur5.go_to_joint(lst_joint_angles_1)

    movebase_client(way_points[11])
    movebase_client(way_points[34])
    movebase_client(way_points[35])

    #################################### Adhesive ##################################
    ur5.go_to_joint([0, -0.47, -0.785, -1, -0.72, 1.57])

    object_ids, object_tranforms  = findObjects()

    if object_ids[6]!=-1:
        x, y, z = 0.01, -0.18, 0.27
        if object_ids[6]==75:
            x, y, z = 0.01, -0.18, 0.27
        elif object_ids[2]==73:
            x, y, z = -0.015, -0.17, 0.27

        ur5_pose_1 = geometry_msgs.msg.Pose()
        trans = object_tranforms[6][0]
        ur5_pose_1.position.x = trans[0]+x
        ur5_pose_1.position.y = trans[1]+y
        ur5_pose_1.position.z = trans[2]+z
        angles = quaternion_from_euler(3.8, 0, -3.14)
        ur5_pose_1.orientation.x = angles[0]
        ur5_pose_1.orientation.y = angles[1]
        ur5_pose_1.orientation.z = angles[2]
        ur5_pose_1.orientation.w = angles[3]
        ur5.go_to_pose(ur5_pose_1)

        ur5_pose_1.position.z = trans[2]+0.15
        ur5.go_to_pose(ur5_pose_1)
        ur5.closeGripper(0.3)
        ur5.go_to_joint([0.5, -0.37, -0.785, -1, -0.65, 1.57])
        print(str(names[6]) + " Picked")

    ur5.go_to_joint(lst_joint_angles_2)

    movebase_client(way_points[37])
    movebase_client(way_points[14])
    movebase_client(way_points[15])

    ############################### Conference room #############################
    flag_object = np.ones(8)
    for i in range(24,26):
        movebase_client(way_points[i])

    print("Conference Room Reached")

    state=[0, 0, -0.8, 0, 1.36, 0]

    ur5.go_to_joint(state)

    state=[0.6, 0, -0.8, 0, 0.5, 0]
    ur5.go_to_joint(state)

    ur5.openGripper()
    print(str(names[6]) + " Dropped in Dropbox 1")

    state=[0, 0, -0.8, 0, 0, 0]
    ur5.go_to_joint(state)

    ur5.go_to_joint(lst_joint_angles_1)

    movebase_client(way_points[26])
    movebase_client(way_points[27])
    movebase_client(way_points[28])
    movebase_client(way_points[38])

    cv2.destroyAllWindows()
    print("Mission Accomplished!")
    del ur5

if __name__ == '__main__':
    main()
