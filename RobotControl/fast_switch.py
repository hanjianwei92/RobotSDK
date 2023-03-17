import multiprocessing
import time
import json
import numpy as np
from typing import Union
from RobotControl import RobotNode, DobotControl, RobotMoveStyle, UniversalGraspHandCtl
from pathlib import Path
from scipy.spatial.transform import Rotation


class FastSwitcher:
    def __init__(self,
                 fs_pose_json_path: str,
                 robot_init_pos_joint: str,
                 robot_ctl: Union[RobotNode, DobotControl]):
        self.robot = robot_ctl
        self.fs_pose_json_path = fs_pose_json_path
        self.air_ctl = UniversalGraspHandCtl()

        self.fs_ready_pose_joint = None

        self.having_switcher = None
        self.fs_pose = None

        try:
            with open(fs_pose_json_path) as f:
                self.fs_pose = json.load(f)
            self.fs_ready_pose_joint = tuple(np.loadtxt(robot_init_pos_joint))
            self.having_switcher = True
        except Exception as e:
            print(f"加载位姿错误 {e}")
            self.having_switcher = False

    def move_joint(self, joint_list, move_style=RobotMoveStyle.move_joint, check_joints_range=None):
        if isinstance(self.robot, RobotNode):
            return self.robot.move_joint(joint_list, move_style, check_joints_range)
        else:
            self.robot.move_to_waypoints_in_joint(joint_list, move_style, check_joints_range)
            return True

    def move_pose(self, pose_list, coor=0, move_style=0, offset=0.0, is_init=True, check_joints_range=None):
        if isinstance(self.robot, RobotNode):
            return self.robot.move_pose(pose_list, coor, move_style, offset, is_init, check_joints_range)
        else:
            self.robot.move_to_waypoints(waypoints=pose_list,
                                         coor=coor,
                                         move_style=move_style,
                                         offset=offset,
                                         check_joints_degree_range=check_joints_range)
            return True

    def get_save_fs_pose_json(self):
        fs_pose_dict = dict()
        print("please input number of fast switcher: ")
        key = input()
        if int(key) not in range(1, 10):
            print("input error")
            return
        for num in range(1, int(key) + 1):
            print(f"press enter to get fs_store_{num} pose")
            input()
            _, pos, ori = self.robot.get_current_waypoint()
            fs_pose_dict[f"fs_store_{num}"] = {"pos": pos, "ori": ori}
            print(f"press enter to get fs_middle_{num} pose")
            input()
            _, pos, ori = self.robot.get_current_waypoint()
            fs_pose_dict[f"fs_middle_{num}"] = {"pos": pos, "ori": ori}

        # dict保存为json文件
        with open(self.fs_pose_json_path, "w") as f:
            json.dump(fs_pose_dict, f, indent=2)

    @staticmethod
    def pos_to_rmatrix(pos: list, ori: list):
        """
        位姿转换为旋转平移矩阵
        :param pos: 位置[x,y,z]
        :param ori: 姿态[Rx, Ry, Rz]
        :return: RT旋转平移矩阵
        """
        rt_mat = np.eye(4)
        r_matrix = Rotation.from_euler('xyz', ori, degrees=True).as_matrix()
        rt_mat[:3, :3] = r_matrix
        rt_mat[:3, 3:] = np.reshape(pos, (3, 1))
        return rt_mat

    def release_switcher(self, num):
        try:
            self.move_joint([self.fs_ready_pose_joint])
            pose_rt_middle = self.pos_to_rmatrix(self.fs_pose[f"fs_middle_{num}"]["pos"],
                                                 self.fs_pose[f"fs_middle_{num}"]["ori"])

            pose_rt_store = self.pos_to_rmatrix(self.fs_pose[f"fs_store_{num}"]["pos"],
                                                self.fs_pose[f"fs_store_{num}"]["ori"])

            ret = self.move_pose([pose_rt_middle], offset=0.40, is_init=False)
            if ret is False:
                raise Exception("error")
            self.robot.move_offset(0.40)

            ret = self.move_pose([pose_rt_store], offset=0.01, is_init=False)
            if ret is False:
                raise Exception("error")
            self.robot.move_offset(0.01)

            self.air_ctl.grasp()
            self.robot.move_offset(-0.10)
            self.air_ctl.release()

            return True

        except Exception as e:
            print(f"release grasper failed , {e}")
            self.move_joint([self.fs_ready_pose_joint])
            return False

    def connect_switcher(self, num):
        try:
            pose_rt_middle = self.pos_to_rmatrix(self.fs_pose[f"fs_middle_{num}"]["pos"],
                                                 self.fs_pose[f"fs_middle_{num}"]["ori"])

            pose_rt_store = self.pos_to_rmatrix(self.fs_pose[f"fs_store_{num}"]["pos"],
                                                self.fs_pose[f"fs_store_{num}"]["ori"])

            ret = self.move_pose([pose_rt_store], offset=0.10, is_init=False)
            if ret is False:
                raise Exception("error")
            self.air_ctl.grasp()
            self.robot.move_offset(0.10)
            self.air_ctl.release()
            time.sleep(1)
            self.robot.move_offset(-0.01)

            ret = self.move_pose([pose_rt_middle], offset=0.0, is_init=False)
            if ret is False:
                raise Exception("error")
            self.robot.move_offset(-0.40)

            self.move_joint([self.fs_ready_pose_joint])

            return True

        except Exception as e:
            print(f"connect grasper failed , {e}")
            self.move_joint([self.fs_ready_pose_joint])
            return False


if __name__ == "__main__":
    robot = RobotNode(info_queue=multiprocessing.Queue(), error_queue=multiprocessing.Queue(),
                      tool_end=dict(pos=(0, 0, 0), ori=(0, 0, 0)),
                      robot_brand="dobot")
    fs = FastSwitcher(str(Path(__file__).parent / "config/fs_pose_json.json"),
                      str(Path(__file__).parent / f"config/{robot.robot_brand}/init_pos_joint.txt"),
                      robot)
    time.sleep(5)
    if fs.release_switcher(1):
        fs.connect_switcher(2)
