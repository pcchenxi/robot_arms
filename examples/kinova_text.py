from robot_arms.kinova.kinova_basic import Kinova

kinova = Kinova()
joint = kinova.get_joint_pose()
# kinova.set_gripper_opening(0.05)
# print(joint)

kinova.check_fk()
