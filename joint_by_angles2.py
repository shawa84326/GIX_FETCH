#!/usr/bin/env python

# forward_kinematics.py: Move the fetch arm to a desired pose using forward kinematics
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

# Note: fetch_moveit_config move_group.launch must be running
# Safety!: Do NOT run this script near people or objects.
# Safety!: There is NO perception.
#          The ONLY objects the collision detection software is aware
#          of are itself & the floor.
if __name__ == '__main__':
    rospy.init_node("forward_kinematics_example")

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm", "base_link")

    # Define ground plane
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

    # TF joint names
    joint_names = ["shoulder_pan_joint",
                   "shoulder_lift_joint", "upperarm_roll_joint",
                   "elbow_flex_joint", "forearm_roll_joint",
                   "wrist_flex_joint", "wrist_roll_joint"]

    # Specify your desired joint angles (modify these values accordingly)
    desired_joint_angles = [1.5, 1.05, 1.43, -1.29, -0.72, -1.61, -0.09]

    # Move the arm to the desired joint angles
    move_group.moveToJointPosition(joint_names, desired_joint_angles, wait=True)

    # Wait for the result
    move_group.get_move_action().wait_for_result()
    result = move_group.get_move_action().get_result()

    if result:
        # Checking the MoveItErrorCode
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Arm moved to the desired joint angles!")
        else:
            rospy.logerr("Arm goal in state: %s", move_group.get_move_action().get_state())
    else:
        rospy.logerr("MoveIt! failure no result returned.")

    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    move_group.get_move_action().cancel_all_goals()
