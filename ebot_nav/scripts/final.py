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

from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_ros_planning_interface import _moveit_planning_scene_interface
from geometry_msgs.msg import PoseStamped, Pose, Point
from object_msgs.msg import ObjectPose
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
import pyassimp

import cv2
import cv_bridge

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

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

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

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

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
        if flag_close:
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Cannot close" + '\033[0m')

        return flag_close

    def go_to_joint(self, joint_location):
        self._group.set_joint_value_target(joint_location) #[1.1, -0.9, 1.28, -1.92, -1.54, -0.21]
        flag_plan = self._group.go(wait=True)

    def openGripper(self):
        self.gripper.set_named_target("open")
        flag_open = self.gripper.go(wait=True)
        if flag_open:
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Cannot close" + '\033[0m')

        return flag_open

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def findObjects():
    listener = tf.TransformListener()
    coke, glue, battery = False, False, False
    tran_coke, tran_glue, tran_battery = 0, 0, 0
    rate = rospy.Rate(10.0)
    obj_name = '/object_'
    while not rospy.is_shutdown():
        for i in range(25, 40):
            try:
                obj_id = obj_name + str(i)
                (tran1, _) = listener.lookupTransform('/ebot_base', obj_id, rospy.Time(0))
                (tran2, _) = listener.lookupTransform('/base_link', obj_id, rospy.Time(0))
                if 25<=i<=29:
                    tran_coke1 = tran1
                    tran_coke2 = tran2
                    coke = True
                elif 34<=i<=37:
                    tran_battery1 = tran1
                    tran_battery2 = tran2
                    battery = True
                elif 30<=i<=33 or i==39:
                    tran_glue1 = tran1
                    tran_glue2 = tran2
                    glue = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        if coke and glue and battery:
            break
        rate.sleep()
    return tran_coke1, tran_glue1, tran_battery1, tran_coke2, tran_glue2, tran_battery2

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

def main():
    global ur5
    ur5 = Ur5Moveit()

    # global detection_pub
    # detection_pub = rospy.Publisher('/detection_info', ObjectPose, queue_size=10)
    # state=[0, -0.07, 0.03, -0.25, 0, -0.21]
    state=[0, -0.37, -0.785, -1, -0.52, 1.57]
    ur5.go_to_joint(state)

    rospy.sleep(10.0)

    state=[0, -0.37, -0.785, -1, -0.7, 1.57]
    ur5.go_to_joint(state)
    # rospy.sleep(5.0)
    # state=[0, -0.37, -0.785, -1, -0.8, 1.57]
    # ur5.go_to_joint(state)
    # rospy.sleep(5.0)

    '''
    coke_pos, glue_pos, battery_pos, coke_pos1, glue_pos1, battery_pos1 = findObjects()
    print("coke", coke_pos)
    print("glue", glue_pos)
    print("milk", battery_pos)

    addObjectBox("coke", coke_pos, (0.04, 0.04, 0.12), (0, 0.02, 0), (0, 0, 0))
    addObjectBox("glue", glue_pos, (0.04, 0.02, 0.12), (0, 0.01, 0) , (0, 0, 0))
    addObjectBox("battery", battery_pos, (0.03, 0.02, 0.1), (-0.01, 0.022, 0) , (0, 0, -0.6))

    get_publish_obj("coke", coke_pos1)
    get_publish_obj("battery", battery_pos1)
    get_publish_obj("glue", glue_pos1)

    ############################################
    #Coke code start
    right, left_back, right_back, middle_back, left = False, False, False, False, False
    state=[0, -0.37, -0.785, -1, -0.52, 1.57]
    xx = 0
    if coke_pos[0]<glue_pos[0] and coke_pos[0]<battery_pos[0]:
        state=[0, -0.47, 0.03, -1.57, -0.52, 1.57]  #left
        xx = -0.01
        left = True
        if coke_pos[1]>glue_pos[1] and coke_pos[1]>battery_pos[1]:
            xx = -0.005     #Back
            left_back = True
    elif coke_pos[0]>glue_pos[0] and coke_pos[0]>battery_pos[0]:
        xx = 0.007          #right
        right = True
        if coke_pos[1]>glue_pos[1] and coke_pos[1]>battery_pos[1]:
            xx = 0.004      #Back
            right_back = True
    elif coke_pos[1]>glue_pos[1] and coke_pos[1]>battery_pos[1]:
        middle_back = True  #Middle

    ur5.go_to_joint(state)

    if not right_back:
        print("Going to top of coke")
        ur5_pose_1 = geometry_msgs.msg.Pose()
        ur5_pose_1.position.x = coke_pos[0]+xx
        ur5_pose_1.position.y = coke_pos[1]-0.098
        if left_back:
            ur5_pose_1.position.z = coke_pos[2]+0.28
        else:
            ur5_pose_1.position.z = coke_pos[2]+0.287
        angles = quaternion_from_euler(4.18, 0, -3.14)
        ur5_pose_1.orientation.x = angles[0]
        ur5_pose_1.orientation.y = angles[1]
        ur5_pose_1.orientation.z = angles[2]
        ur5_pose_1.orientation.w = angles[3]
        ur5.go_to_pose(ur5_pose_1)

    print("Going to coke")
    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = coke_pos[0]+xx
    ur5_pose_1.position.y = coke_pos[1]-0.098
    ur5_pose_1.position.z = coke_pos[2]+0.245
    angles = quaternion_from_euler(4.18, 0, -3.14)
    ur5_pose_1.orientation.x = angles[0]
    ur5_pose_1.orientation.y = angles[1]
    ur5_pose_1.orientation.z = angles[2]
    ur5_pose_1.orientation.w = angles[3]
    ur5.go_to_pose(ur5_pose_1)

    print("Closing gripper")
    if middle_back:
        ur5.closeGripper(0.23)
    else:
        ur5.closeGripper(0.22)

    if right and not right_back:
        print("Going to drop location")
        state = [-0.19, 0.0047, -0.556, -1.431, -1.049, 1.35]
        ur5.go_to_joint(state)
    else:
        print("Going up")
        state = [1.0318, 0.331, -0.788, -3.21, -1.03, 3.14]
        ur5.go_to_joint(state)

        print("Going to drop location")
        state = [-0.19, 0.0047, -0.556, -1.431, -1.049, 1.35]
        ur5.go_to_joint(state)

    print("Opening gripper")
    ur5.openGripper()
    print("Gripper opened")
    #Coke code end

    #############################################################################
    #Battery code start
    right = False
    left = False
    right_back = False
    state=[0, -0.37, -0.785, -1, -0.52, 1.57]
    if coke_pos[0]<=battery_pos[0]<=glue_pos[0] or glue_pos[0]<=battery_pos[0]<=coke_pos[0]:
        tolerance_y = 0.1   #Middle
        gripper_val = 0.32
    elif battery_pos[0]>=glue_pos[0] and battery_pos[0]>=coke_pos[0]:
        tolerance_y = 0.1   #right
        gripper_val = 0.32
        right = True
        if not (battery_pos[1]>=coke_pos[1] and battery_pos[1]>=glue_pos[1]):
            right_back = True
            tolerance_y = 0.11
    else:
        state=[0, -0.47, 0.03, -1.57, -0.52, 1.57]
        tolerance_y = 0.1   #left
        gripper_val = 0.32
        left = True

    ur5.go_to_joint(state)

    if left:
        print("Going to top of battery")
        ur5_pose_1 = geometry_msgs.msg.Pose()
        ur5_pose_1.position.x = battery_pos[0]
        ur5_pose_1.position.y = battery_pos[1]-tolerance_y
        ur5_pose_1.position.z = battery_pos[2]+0.28
        angles = quaternion_from_euler(4.18, 0, -3.14)
        ur5_pose_1.orientation.x = angles[0]
        ur5_pose_1.orientation.y = angles[1]
        ur5_pose_1.orientation.z = angles[2]
        ur5_pose_1.orientation.w = angles[3]
        ur5.go_to_pose(ur5_pose_1)

    print("Going to battery")
    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = battery_pos[0]
    ur5_pose_1.position.y = battery_pos[1]-tolerance_y
    ur5_pose_1.position.z = battery_pos[2]+0.245
    angles = quaternion_from_euler(4.18, 0, -3.14)
    ur5_pose_1.orientation.x = angles[0]
    ur5_pose_1.orientation.y = angles[1]
    ur5_pose_1.orientation.z = angles[2]
    ur5_pose_1.orientation.w = angles[3]
    ur5.go_to_pose(ur5_pose_1)

    print("Closing gripper")
    ur5.closeGripper(0.306)

    if not battery_pos[0]>glue_pos[0]:
        print("Going up")
        ur5_pose_1 = geometry_msgs.msg.Pose()
        ur5_pose_1.position.x = battery_pos[0]
        ur5_pose_1.position.y = battery_pos[1]-tolerance_y
        ur5_pose_1.position.z = battery_pos[2]+0.28
        angles = quaternion_from_euler(3.14, 0, -3.14)
        ur5_pose_1.orientation.x = angles[0]
        ur5_pose_1.orientation.y = angles[1]
        ur5_pose_1.orientation.z = angles[2]
        ur5_pose_1.orientation.w = angles[3]
        ur5.go_to_pose(ur5_pose_1)

    print("Going to drop location")
    state = [-0.19, 0.0047, -0.556, -1.431, -1.049, 1.35]
    ur5.go_to_joint(state)

    print("Opening gripper")
    ur5.openGripper()
    print("Gripper opened")
    #Battery end state

    ###############################################################
    #Glue code start
    state=[0, -0.37, -0.785, -1, -0.52, 1.57]
    left = False
    if glue_pos[0]<coke_pos[0] and glue_pos[0]<battery_pos[0]:
        state=[0, -0.47, 0.03, -1.57, -0.52, 1.57] #left
        left = True

    ur5.go_to_joint(state)

    print("Going to top of glue")
    ur5_pose_1 = geometry_msgs.msg.Pose()
    if glue_pos[0] < coke_pos[0] and glue_pos[0] < battery_pos[0] :
        ur5_pose_1.position.x = glue_pos[0] - 0.01
    else:
        ur5_pose_1.position.x = glue_pos[0] - 0.0022
    ur5_pose_1.position.y = glue_pos[1] - 0.19
    ur5_pose_1.position.z = glue_pos[2] + 0.2
    angles = quaternion_from_euler(3.68, 0, -3.14)
    ur5_pose_1.orientation.x = angles[0]
    ur5_pose_1.orientation.y = angles[1]
    ur5_pose_1.orientation.z = angles[2]
    ur5_pose_1.orientation.w = angles[3]
    if left:
        ur5.go_to_pose(ur5_pose_1)

    print("Going to glue")
    ur5_pose_1.position.z = glue_pos[2]+ 0.125
    ur5.go_to_pose(ur5_pose_1)

    print("Closing gripper")
    ur5.closeGripper(0.31)

    if left:
        print("Going up")
        state = [1.0318, 0.331, -0.788, -3.21, -1.03, 3.14]
        ur5.go_to_joint(state)

    print("Going to drop location")
    state = [-0.19, 0.0047, -0.556, -1.431, -1.049, 1.35]
    ur5.go_to_joint(state)
    print("Dropping")

    ur5.openGripper()
    #Glue code end
    '''
    del ur5

if __name__ == '__main__':
    main()
