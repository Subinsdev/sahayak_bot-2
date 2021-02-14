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

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import time

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
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

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
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
#
        #angles = euler_from_quaternion([pose_values.orientation.x, pose_values.orientation.y,pose_values.orientation.z,pose_values.orientation.w])
        #print("CURRENT ANGLE")
        #rospy.loginfo(angles)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
#
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

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

# def findObjects(object_ids):
#     listener = tf.TransformListener()
#     coke, glue, battery = False, False, False
#     tran_coke, tran_glue, tran_battery = 0, 0, 0
#     rate = rospy.Rate(10.0)
#     obj_name = '/object_'
    
    # start_time = time.time()
    # end_time = time.time()
    # tran1, tran2, found, idx = 0, 0, False, 0
    # while end_time-start_time<5:
    #     for x in object_ids:
    #         obj_id = obj_name + str(x)
    #         try:
    #             (tran1, _) = listener.lookupTransform('/ebot_base', obj_id, rospy.Time(0))
    #             (tran2, _) = listener.lookupTransform('/base_link', obj_id, rospy.Time(0))
    #             found = True
    #             idx = x
    #             break
    #         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #             continue
    #     if found:
    #         break
    #     rate.sleep()
    #     end_time = time.time()
    #     print("Time completed: ", end_time-start_time)
    # return tran1, tran2, found, idx

def findObjects():
    listener = tf.TransformListener()
    return_object_id = [-1]*8
    return_object_tranform = [0]*8
    rate = rospy.Rate(10.0)
    obj_name = '/object_'
    object_ids = [[59, 71, 78], [57, 74, 76, 82], [55, 70, 73, 75, 83], [56, 41, 80], [58, 43, 64], [44, 68], [42, 72, 81], [45, 48, 49, 50, 51, 52, 53, 66, 67]]
    #0: Wheels, 1: EYFI Board, 2: FPGA, 3: Battery, 4: Glue, 5: Coke, 6: Adhesive, 7: Glass
    start_time = time.time()
    end_time = time.time()
    while end_time-start_time<5:
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
        print("Time completed: ", end_time-start_time)
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
    paths = ["/home/ubuntu1804/eyantra_ws/src/sahayak_bot/ebot_gazebo/models/robot_wheels/meshes/robot_wheels.dae",
    "/home/ubuntu1804/eyantra_ws/src/sahayak_bot/ebot_gazebo/models/eYIFI/meshes/eyifi.dae",
    "/home/ubuntu1804/eyantra_ws/src/sahayak_bot/ebot_gazebo/models/biscuits/meshes/biscuits.dae",
    "/home/ubuntu1804/eyantra_ws/src/sahayak_bot/ebot_gazebo/models/soap2/meshes/soap2.dae",
    "/home/ubuntu1804/eyantra_ws/src/sahayak_bot/ebot_gazebo/models/glue/meshes/glue.dae",
    "/home/ubuntu1804/eyantra_ws/src/sahayak_bot/ebot_gazebo/models/coke_can/meshes/coke_can.dae",
    "/home/ubuntu1804/eyantra_ws/src/sahayak_bot/ebot_gazebo/models/adhesive/meshes/adhesive.dae",
    "/home/ubuntu1804/eyantra_ws/src/sahayak_bot/ebot_gazebo/models/water_glass/meshes/glass.dae"]
    scale = [(1, 1, 1), (0.1, 0.1, 0.1), (1, 1, 1), (1, 1, 1), (1, 1, 1), (1, 1, 1), (1, 1, 1), (1, 1, 1)]
    for i in range(len(object_ids)):
        if object_ids[i]!=-1:
            trans = object_tranforms[i][0]
            obj.pose.position.x = trans[0]
            obj.pose.position.y = trans[1]
            obj.pose.position.z = trans[2]
            ur5.add_mesh(names[i], obj, paths[i], scale[i])    

def main():
    global ur5
    ur5 = Ur5Moveit()

    # global detection_pub
    # detection_pub = rospy.Publisher('/detection_info', ObjectPose, queue_size=10)
    # state=[0, -0.07, 0.03, -0.25, 0, -0.21]
    #Go up and look down
    #state=[0, -0.37, -0.785, -1, -0.52, 1.57]
    states=[[0, -0.37, -0.785, -1, -0.52, 1.57], [0.56, -0.37, -0.785, -1, -0.65, 1.57]]
    
    for state in states:
        ur5.go_to_joint(state)
        object_ids, object_tranforms  = findObjects()
        print(object_ids)
        print(object_tranforms)
        print("Adding deteced objects in rviz")
        add_dected_objects_mesh_in_rviz(ur5, object_ids, object_tranforms)
        print("Done")
        if object_ids[2]!=-1:
            print("Found object_", object_ids[2])
            while not rospy.is_shutdown():
                x = float(input("Enter x: "))
                y = float(input("Enter y: "))
                z = float(input("Enter z: "))
                rot_angle = float(input("Enter rotation: "))
                ur5_pose_1 = geometry_msgs.msg.Pose()
                trans = object_tranforms[2][0]
                ur5_pose_1.position.x = trans[0]+x
                ur5_pose_1.position.y = trans[1]+y
                ur5_pose_1.position.z = trans[2]+z
                angles = quaternion_from_euler(3.8, 0, -3.14+rot_angle)
                ur5_pose_1.orientation.x = angles[0]
                ur5_pose_1.orientation.y = angles[1]
                ur5_pose_1.orientation.z = angles[2]
                ur5_pose_1.orientation.w = angles[3]
                ur5.go_to_pose(ur5_pose_1)
                flag = int(input("Close the gripper: "))
                if flag==1:
                    break
            ur5.closeGripper(0.15)
            ur5.go_to_joint([0.56, -0.37, -0.785, -1, -0.65, 1.57])
            ur5.openGripper()
            break
    print("Should go front?", found)

    #Go forward if you didnt find the object
    # if object_ids[2]==-1:
    #     try:
    #         print("Tryning to go forward")
    #         result = movebase_client([25.8179, -3.2344, -0.8869, 0.46182])
    #         if result:
    #             rospy.loginfo("Goal execution done!")
    #     except rospy.ROSInterruptException:
    #         rospy.loginfo("Navigation test finished.")

    #     for state in states:
    #         ur5.go_to_joint(state)
    #         object_ids, object_tranforms  = findObjects()
    #         print(object_ids)
    #         print(object_tranforms)
    #         print("Adding deteced objects in rviz")
    #         add_dected_objects_mesh_in_rviz(ur5, object_ids, object_tranforms)
    #         print("Done")
    #         if object_ids[2]!=-1:
    #             print("Found object_", object_ids[2])
    #             while not rospy.is_shutdown():
    #                 x = float(input("Enter x: "))
    #                 y = float(input("Enter y: "))
    #                 z = float(input("Enter z: "))
    #                 rot_angle = float(input("Enter rotation: "))
    #                 ur5_pose_1 = geometry_msgs.msg.Pose()
    #                 trans = object_tranforms[2][0]
    #                 ur5_pose_1.position.x = trans[0]+x
    #                 ur5_pose_1.position.y = trans[1]+y
    #                 ur5_pose_1.position.z = trans[2]+z
    #                 angles = quaternion_from_euler(3.8, 0, -3.14+rot_angle)
    #                 ur5_pose_1.orientation.x = angles[0]
    #                 ur5_pose_1.orientation.y = angles[1]
    #                 ur5_pose_1.orientation.z = angles[2]
    #                 ur5_pose_1.orientation.w = angles[3]
    #                 ur5.go_to_pose(ur5_pose_1)
    #                 flag = int(input("Close the gripper: "))
    #                 if flag==1:
    #                     break
    #             ur5.closeGripper(0.15)
    #             ur5.go_to_joint([0.56, -0.37, -0.785, -1, -0.65, 1.57])
    #             ur5.openGripper()
    #             break
    
    print("Finished")

    #Conference room
    # state=[0, 0, -0.8, 0, 0, 0]
    # ur5.go_to_joint(state)

    # state=[0.6, 0, -0.8, 0, 0, 0]
    # ur5.go_to_joint(state)

    # Research lab
    # state=[0, 0, -1.22, 0, 0.96, 0.4]
    # ur5.go_to_joint(state)

    # state=[-1.47, 0, -1.22, 0, 0.96, 0.4]
    # ur5.go_to_joint(state)

    #Metting room
    # state=[0, 0.23, -1.22, 0, 1.36, 0]
    # ur5.go_to_joint(state)

    # state=[1.13, 0.4, -1.22, 0, 1.36, 0]
    # ur5.go_to_joint(state)
    
    #rospy.sleep(10.0)

    # state=[0, -0.37, -0.785, -1, -0.7, 1.57]
    # ur5.go_to_joint(state)
    # rospy.sleep(5.0)
    # state=[0, -0.37, -0.785, -1, -0.8, 1.57]
    # ur5.go_to_joint(state)
    # rospy.sleep(5.0)

    del ur5

if __name__ == '__main__':
    main()
