#! /usr/bin/env python

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
        #rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        #rospy.loginfo(pose_values)
#
        #angles = euler_from_quaternion([pose_values.orientation.x, pose_values.orientation.y,pose_values.orientation.z,pose_values.orientation.w])
        #print("CURRENT ANGLE")
        #rospy.loginfo(angles)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        #pose_values = self._group.get_current_pose().pose
        #rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        #rospy.loginfo(pose_values)
#
        #list_joint_values = self._group.get_current_joint_values()
        #rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        #rospy.loginfo(list_joint_values)
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
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

    def remove_world_obj(self, name = None):
        co =  CollisionObject()
        co.operation =  CollisionObject.REMOVE
        if name != None:
            co.id = name
        self._pub_co.publish(co)

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
    # print("way Point:    ",g)
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def move_base_points(lt):
    for i in range(len(lt)):
        # print(lt[i])
        movebase_client(lt[i])

def findObjects():
    listener = tf.TransformListener()
    return_object_id = [-1]*8
    return_object_tranform = [0]*8
    rate = rospy.Rate(20.0)
    obj_name = '/object_'
    object_ids = [[59, 71, 78, 89, 94, 97, 99, 107], [57, 74, 76, 82, 103], [55, 70, 73, 75, 83, 88], [56, 41, 80, 98, 102, 104], [58, 43, 64, 85, 93, 96], [44, 68, 84, 87, 90], [42, 72, 81, 86, 92, 95, 100, 101, 106], [45, 48, 49, 50, 51, 52, 53, 66, 67, 91]]
    #0: Wheels, 1: EYFI Board, 2: FPGA, 3: Battery, 4: Glue, 5: Coke, 6: Adhesive, 7: Glass
    start_time = time.time()
    end_time = time.time()
    while end_time-start_time<1.4:
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
    names = ["wheels", "eyfi", "fpga", "battery", "glue", "coke", "adhesive", "glass"]
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
    names = ["wheels", "eyfi", "fpga", "battery", "glue", "coke", "adhesive", "glass"]
    for i in range(len(object_ids)):
        if object_ids[i]!=-1:
            ur5.remove_world_obj(names[i])

def showDetectedObjects(msg):
    data = msg.objects.data
    detected_objects_image = image.copy()
    names = ["Wheels", "EYFI", "FPGA", "Battery", "Glue", "Coke", "Adhesive", "Glass"]
    object_ids = [[59, 71, 78, 89, 94, 97, 99, 107], [57, 74, 76, 82, 103], [55, 70, 73, 75, 83, 88], [56, 41, 80, 98, 102, 104], [58, 43, 64, 85, 93, 96], [44, 68, 84, 87, 90], [42, 72, 81, 86, 92, 95, 100, 101, 106], [45, 48, 49, 50, 51, 52, 53, 66, 67, 91]]
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
                detected_objects_image = cv2.polylines(detected_objects_image, [detected[idx]], True, (255, 0, 0), 3)
                [x, y] = detected[idx][0][0]
                org = (x+10, y+10)
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

way_points_in = {
                'SZ':[
                    ( 0,0,-0.719, -0.694)
                    ],
                'MR':[
                    ( 8.64, 1.148, 0.7063, 0.7078), #10   Meeting Intermediate CV
                    ( 8.64, 2.4, 1.0, 0.0003), #11  Meeting Intermediate 2 CV
                    ( 6.9, 2.6, 0.0, 0.010), #12   Meeting DropBox
                    ( 7.6, 2.4, 0.0, 0.007) #13   Meeting Pickup

                    ],
                'P':[
                    (13.035, 0.99, -0.702, 0.711), #1  Pantry Intermediate outside door
                    (13.035, -0.1, -0.702, 0.711), #2  Pantry Intermediate inside door
                    (13.035, -0.1, -0.182, 0.98), #3   Pantry Intermediate inside door orientation
                    (14.695, -0.75, -0.718, 0.695), #4  Pantry Pickup Table 1
                    (14.615, -0.62, 1, 0.0008), #5   Pantry Pickup Table 1 - Orientation to Table2
                    (11.15, -1.32, -0.709, -0.70), #6   Pantry Pickup Table 2 Position 1
                    (11.15, -1.32, 0.3824, 0.92395) #7  Pantry Pick.70up Table 2 Position 1 Orientation

                ],
                'RL':[
                    (15.0, 3.9, -0.666, -0.74542),
                    (14.8, 10.237 , -0.9999 , -0)
                ],
                'SR':[
                    (16.7,1.0,0,1),
                    (26.065, -2.714, -0.891, 0.454), #19  Store Pickup 1
                    # (26.065, -2.714, -0.994, 0.0),  #20   Store Pickup 1 Orientation to Exit
                    (25.8179, -3.3344, -0.871, 0.490) #21   Store Pickup 2
                ],
                'CR':[
                    ( 5.156, 0.861, -0.706, 0.7082), #24   Conference Intermediate CV
                    ( 5.070, -0.7, -0.9126, 0.4087) #25  Conference DropBox CV
                ]
                }

way_points_out = {
                'SZ':[
                    ( 3.6, 0.85,0,1) #0  enter the hallway point
                    ],
                'MR':[
                    ( 8.5, 2.175, -0.7068, 0.7073), #14   Meeting Intermediate out CV
                    ( 8.638, 1.148, -0.7068, 0.7073) #15   Meeting Intermediate CV
                    ],
                'RL':[
                    (11.2, 10.010, -0.91537, 0.402)
                ],
                'SR':[
                    (25.8179, -3.3344, 0.894, -0.448) #22    Store Pickup 2 Orientation to Exit
                ],
                'P':[
                        ( 13.2159, -0.604, -0.7577, -0.650), #8  Pantry Out OIntermediate
                        ( 13.0, 0.8, -0.719, -0.694) #9  Pantry Out OIntermediate
                ],
                'CR':[
                    ( 5.070, -0.7, -0.7583, -0.6518), #26  Conference Intermediate out CV
                    ( 5.18, 0.1, -0.7583, -0.6518), #27   Conference Intermediate out CV 1
                    ( 5.25, 0.65, -0.7583, -0.6518) #28   Conference Intermediate out CV 2
                ]
                }

dropbox_points = {
                'MR':[
                    ( 8.5, 2.175, -0.7068, 0.7073), #14   Meeting Intermediate out CV
                    ( 8.638, 1.148, -0.7068, 0.7073) #15   Meeting Intermediate CV
                    ],
                'RL':[
                    (11.2, 10.010, -0.91537, 0.402)
                ],
                'CR':[
                    ( 5.070, -0.7, -0.7583, -0.6518), #26  Conference Intermediate out CV
                    ( 5.18, 0.1, -0.7583, -0.6518), #27   Conference Intermediate out CV 1
                    ( 5.25, 0.65, -0.7583, -0.6518) #28   Conference Intermediate out CV 2
                ]
}


pickup_points = {
                'SZ':[
                    ( 3.6, 0.85,0,1) #0  enter the hallway point
                    ],
                'MR':[
                    ( 8.5, 2.175, -0.7068, 0.7073), #14   Meeting Intermediate out CV
                    ( 8.638, 1.148, -0.7068, 0.7073) #15   Meeting Intermediate CV
                    ],
                'RL':[
                    (11.2, 10.010, -0.91537, 0.402)
                ],
                'SR':[
                    (25.8179, -3.3344, 0.894, -0.448) #22    Store Pickup 2 Orientation to Exit
                ],
                'P':[
                        ( 13.2159, -0.604, -0.7577, -0.650), #8  Pantry Out OIntermediate
                        ( 13.0, 0.8, -0.719, -0.694) #9  Pantry Out OIntermediate
                ],
}

Intermediate = {

}


rooms = ['SZ','MR','RL','SR','P','CR']

time_points = np.zeros((6,6))

def move_direct(l1, l2):
    move_base_points(way_points_out[l1])
    if(l1 == 'RL' and l2 == 'SR'):
        movebase_client((14.6, 10.1097, -0.011123, -0.9936))
    if(l2 == 'RL' and (l1 == 'SZ' or l1 == 'CR')):
        movebase_client((10.0, 1.34, -0.149, -0.5))
    # if((l2 == 'RL') and ()):
    move_base_points(way_points_in[l2])
def main():
    global ur5
    ur5 = Ur5Moveit()
    sub2 = rospy.Subscriber('/camera/color/image_raw2', Image, callback=storeImage, queue_size=10)


    # print("Waiting for image")
    while True:
        try:
            image
            break
        except:
            continue
    print("Launched Detectetion Object Window")

    sub1 = rospy.Subscriber('/objectsStamped', ObjectsStamped, callback=showDetectedObjects, queue_size=10)

    names = ["Wheels", "EYFI", "FPGA", "Battery", "Glue", "Coke", "Adhesive", "Glass"]

    lst_joint_angles_1 = [-0.4, 0.74, 0.31, -0.02, -0.96, -0.02] #[0, 1.08, 0.37, 0, 0, 0]
    lst_joint_angles_2 = [-0.5, 0.74, 0.31, -0.02, -0.96, -0.02] #[0, 1.08, 0.37, 0, 0, 0]

    #going to the crouch position
    # while not rospy.is_shutdown():
    f = ur5.go_to_joint(lst_joint_angles_1)
        # if f:
        #     # obj = Pose()
        #     # pub1.publish(obj)
        #     break
        # rospy.sleep(2)
    # movebase_client((0,0,0,0))
    print("Started Run!!")

    # move_base_points(way_points_out['SZ'])
    move_direct('SZ', 'SR')

    #################################### Battery ##################################
    ############################ Looking Left Side  ############################
    states=[[0, -0.37, -0.785, -1, -0.52, 1.57], [0.56, -0.37, -0.785, -1, -0.65, 1.57]]
    for state in states:
        ur5.go_to_joint(state)
        object_ids, object_tranforms  = findObjects()
        for j  in range(8):
            if object_ids[j] != -1:
                print(str(names[j]) + "Identified")
        print("Adding deteced objects in rviz")
        if object_ids[3]!=-1:
            print("Found object_", object_ids[3])
            x, y, z = -0.07, -0.18, 0.19
            if object_ids[2]==101:
                x, y, z = -0.07, -0.18, 0.19
            elif object_ids[2]==80:
                x, y, z = -0.07, -0.18, 0.21
            elif object_ids[2]==102:
                x, y, z = -0.07, -0.18, 0.21
            elif object_ids[2]==56:
                x, y, z = -0.05, -0.18, 0.19

            ur5_pose_1 = geometry_msgs.msg.Pose()
            trans = object_tranforms[3][0]
            ur5_pose_1.position.x = trans[0]+x
            ur5_pose_1.position.y = trans[1]+y
            ur5_pose_1.position.z = trans[2]+z
            angles = quaternion_from_euler(3.8, 0, -3.44)
            ur5_pose_1.orientation.x = angles[0]
            ur5_pose_1.orientation.y = angles[1]
            ur5_pose_1.orientation.z = angles[2]
            ur5_pose_1.orientation.w = angles[3]
            ur5.go_to_pose(ur5_pose_1)

            ur5_pose_1.position.z = trans[2]+0.155
            ur5.go_to_pose(ur5_pose_1)
            ur5.closeGripper(0.45)
            ur5.go_to_joint([0.5, -0.37, -0.785, -1, -0.65, 1.57])
            ur5.go_to_joint([-0.5, -0.37, -0.785, -1, -0.65, 1.57])
            print(str(names[3]) + " Picked")
            ur5.go_to_joint(lst_joint_angles_2)
            movebase_client(way_points[20])
            break

    if object_ids[3]==-1:
        ur5.go_to_joint(lst_joint_angles_2)
        movebase_client(way_points[21])

        #################RIGHT SIDE##################
        states=[[0, -0.37, -0.885, -1, -0.7, 1.57]]
        for state in states:
            ur5.go_to_joint(state)
            object_ids, object_tranforms  = findObjects()
            for j in range(8):
                if object_ids[j] != -1:
                    print(str(names[j]) + " Identified")
            if object_ids[3]!=-1:
                print("Found object_", object_ids[3])
                x, y, z = -0.07, -0.18, 0.19
                if object_ids[2]==103:
                    x, y, z = -0.07, -0.18, 0.19
                elif object_ids[2]==56:
                    x, y, z = -0.045, -0.18, 0.19
                elif object_ids[2]==98:
                    x, y, z = -0.065, -0.18, 0.19

                ur5_pose_1 = geometry_msgs.msg.Pose()
                trans = object_tranforms[3][0]
                ur5_pose_1.position.x = trans[0]+x
                ur5_pose_1.position.y = trans[1]+y
                ur5_pose_1.position.z = trans[2]+z
                angles = quaternion_from_euler(3.8, 0, -3.44)
                ur5_pose_1.orientation.x = angles[0]
                ur5_pose_1.orientation.y = angles[1]
                ur5_pose_1.orientation.z = angles[2]
                ur5_pose_1.orientation.w = angles[3]
                ur5.go_to_pose(ur5_pose_1)

                ur5_pose_1.position.z = trans[2]+0.155
                ur5.go_to_pose(ur5_pose_1)
                ur5.closeGripper(0.45)
                ur5.go_to_joint([0.5, -0.37, -0.785, -1, -0.65, 1.57])
                ur5.go_to_joint([-0.5, -0.37, -0.785, -1, -0.65, 1.57])
                print(str(names[2])+ " Picked")
                ur5.go_to_joint(lst_joint_angles_2)
                movebase_client(way_points[22])
                break


    if(object_ids[3]!=-1):
        ur5.go_to_joint(lst_joint_angles_1)

    move_direct('SR', 'RL')
    move_direct('RL', 'P')
    move_direct('P', 'CR')


    print(time_points)
    cv2.destroyAllWindows()
    print("Start Point Reached!!")
    print("Mission Accomplished")
    del ur5

if __name__ == '__main__':
    main()

####<node pkg="ebot_nav" type=""b.py" name="code" output="screen" launch-prefix="gnome-terminal --command" />
