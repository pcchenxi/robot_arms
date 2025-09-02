from robot_arms.kinova.kinova_basic import Kinova

kinova = Kinova()
joint = kinova.get_joint_pose()

print(joint)
