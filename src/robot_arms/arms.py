from robot_arms.franka.franka_basic import Franka 
from robot_arms.kinova.kinova_basic import Kinova
import numpy as np

LEFT_SHOULDER_SHIFT = np.array([-0.003, 0.313, 0.0])
RIGHT_SHOULDER_SHIFT = np.array([-0.003, -0.313, 0.0])

class FrankaLeft(Franka):
    def __init__(self, robot_ip='192.168.31.11', relative_dynamics_factor=0.04):
        super(FrankaLeft, self).__init__(robot_ip, relative_dynamics_factor)
        self.pose_translate = LEFT_SHOULDER_SHIFT

        # up default
        sup_joint_pose = np.array([0.03714000366948475, -1.0011939601582107, 0.012796065041415793, -1.9821447593672037, -0.3933088893534684, 1.855200764573804, 0.8445985249531525])
        # sup_joint_pose = np.array([0.03714000366948475, -1.0011939601582107, 0.012796065041415793, -1.9821447593672037, -0.3933088893534684, 1.855200764573804, 0.8445985249531525])
        # sup_joint_pose = np.array([0.2323472929627809, 0.16501024922001867, 0.08329819191839495, -1.561295524505366, -0.7199708017554436, 1.7323099317057986, 1.8030975948369037])

        # down default
        start_joint_pose = np.array([-0.7691188308293451, -1.247877516339364, 1.6423489425470328, -2.5036577137882414, -0.31302955191138837, 2.1688551648645142, 2.679632657716602])

        self.start_joint_pose = start_joint_pose
        self.sup_joint_pose = start_joint_pose

class FrankaRight(Franka):
    def __init__(self, robot_ip='192.168.31.12', relative_dynamics_factor=0.05):
        super(FrankaRight, self).__init__(robot_ip, relative_dynamics_factor)
        self.pose_translate = RIGHT_SHOULDER_SHIFT

        start_joint_pose = np.array([0.5166863674038985, -0.9506078305042858, -0.1274808768203859, -1.9297218054954717, 0.05532653038053947, 1.827222911086791, 0.8727408676298193])
        sup_joint_pose = np.array([0.20449955761572036, -0.1585066335656234, -0.5613969538235338, -1.7035633041896554, 0.8227487667154362, 1.5262386113608881, -0.40660795214503953])

        self.start_joint_pose = start_joint_pose
        self.sup_joint_pose = sup_joint_pose

class FrankaDual():
    def __init__(self, left_ip, right_ip, relative_dynamics_factor):
        self.robot_left = FrankaLeft(left_ip, relative_dynamics_factor)
        self.robot_right = FrankaLeft(right_ip, relative_dynamics_factor)

class KinovaFront(Kinova):
    def __init__(self, robot_ip='192.168.31.13'):
        super(KinovaFront, self).__init__()
        self.pose_translate = np.array([0.890, 0.425, 0.03])
        self.pose_rotate = np.array([0.0, 0.0, 1.0, 0.0])
        
        start_joint_pose = np.array([-2.1208216498900514, -0.5922476116494906, -1.4907683360380153, -1.6488727531049676, -0.5771900988461809, -0.8916668130070988, -1.5281298229707367])
        # sup_joint_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.start_joint_pose = start_joint_pose
        self.sup_joint_pose = start_joint_pose
