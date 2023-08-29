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

        # 当前末端执行器编号, -1表示未知, 0表示无末端执行器, 1表示夹爪, 2表示小吸盘, 3表示大吸盘, 4表示标定块
        self.current_switcher_num = self.get_curr_fs_num()

        try:
            with open(fs_pose_json_path) as f:
                self.fs_pose = json.load(f)
            self.fs_ready_pose_joint = tuple(np.loadtxt(robot_init_pos_joint))
            self.having_switcher = True
        except Exception as e:
            print(f"加载位姿错误 {e}")
            self.having_switcher = False

    def get_curr_fs_num(self):
        fs_statues = self.air_ctl.get_fs_statue(1, 4)
        fs_statues = np.array(fs_statues).squeeze()
        if np.sum(fs_statues) == 4:
            return 0
        elif np.sum(fs_statues) == 3:
            return np.argwhere(fs_statues == 0)[0][0] + 1
        else:
            return -1

    def move_joint(self, joint_list, move_style=RobotMoveStyle.move_joint, check_joints_range=None,
                   speed_ratio=0, acc_ratio=0):
        if isinstance(self.robot, RobotNode):
            return self.robot.move_joint(joint_list, move_style, check_joints_range, speed_ratio, acc_ratio)
        else:
            self.robot.move_to_waypoints_in_joint(joint_list, move_style, check_joints_range, speed_ratio, acc_ratio)
            return True

    def move_pose(self, pose_list, coor=0, move_style=0, offset=0.0, is_init=True, check_joints_range=None,
                  speed_ratio=0, acc_ratio=0):
        if isinstance(self.robot, RobotNode):
            return self.robot.move_pose(pose_list, coor, move_style, offset, is_init, check_joints_range,
                                        speed_ratio=speed_ratio, acc_ratio=acc_ratio)
        else:
            self.robot.move_to_waypoints(waypoints=pose_list,
                                         coor=coor,
                                         move_style=move_style,
                                         offset=offset,
                                         check_joints_degree_range=check_joints_range,
                                         speed_ratio=speed_ratio,
                                         acc_ratio=acc_ratio)
            return True

    def get_save_fs_pose_json(self):
        fs_pose_dict = dict()
        print("please input number of fast switcher: ")
        key = input()
        if int(key) not in range(1, 10):
            print("input error")
            return
        for num in range(1, int(key) + 1):
            print(f"please input fast_switcher_{num} pose (x,y,z,rx,ry,rz; unit: meter and degree, "
                  f"relative to robot flange).")
            pose_string = input()
            pose = pose_string.split(",")
            fs_pose_dict[f"tool_end_{num}_pose"] = {"pos": [float(pose[0]), float(pose[1]), float(pose[2])],
                                                    "ori": [float(pose[3]), float(pose[4]), float(pose[5])]}

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
        self.current_switcher_num = self.get_curr_fs_num()
        if self.having_switcher is False or self.current_switcher_num == -1:
            print("don`t set fast switcher num")
            return False
        if self.current_switcher_num != num:
            print("current switcher num error")
            return False
        if self.current_switcher_num == 0:
            print("don`t need release switcher")
            return True
        try:
            self.robot.tool_end = dict(pos=[0.0, 0.0, 0.0], ori=[0.0, 0.0, 0.0])
            self.move_joint([self.fs_ready_pose_joint])
            pose_rt_middle = self.pos_to_rmatrix(self.fs_pose[f"fs_middle_{num}"]["pos"],
                                                 self.fs_pose[f"fs_middle_{num}"]["ori"])

            pose_rt_store = self.pos_to_rmatrix(self.fs_pose[f"fs_store_{num}"]["pos"],
                                                self.fs_pose[f"fs_store_{num}"]["ori"])

            ret = self.move_pose([pose_rt_middle], offset=0.30, is_init=False)
            if ret is False:
                raise Exception("error")
            self.robot.move_offset(0.30)

            ret = self.move_pose([pose_rt_store], offset=0.01, is_init=False, move_style=RobotMoveStyle.move_joint_line)
            if ret is False:
                raise Exception("error")
            self.robot.move_offset(0.01)

            self.air_ctl.grasp()
            time.sleep(0.1)
            self.robot.move_offset(-0.10)
            self.air_ctl.release()

            self.current_switcher_num = self.get_curr_fs_num()

            return True

        except Exception as e:
            print(f"release grasper failed , {e}")
            self.move_joint([self.fs_ready_pose_joint])
            return False

    def connect_switcher(self, num):
        self.current_switcher_num = self.get_curr_fs_num()
        if self.having_switcher is False or self.current_switcher_num == -1:
            print("don`t set fast switcher num")
            return False

        if self.current_switcher_num == num:
            print("don`t need change fast switcher")
            self.robot.tool_end = self.fs_pose[f"tool_end_{num}_pose"]
            return True

        if self.current_switcher_num != 0 and self.current_switcher_num != num:
            # print("please release switcher first")
            self.release_switcher(self.current_switcher_num)

        try:
            self.robot.tool_end = dict(pos=[0.0, 0.0, 0.0], ori=[0.0, 0.0, 0.0])
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
            time.sleep(0.1)
            self.robot.move_offset(-0.01)

            ret = self.move_pose([pose_rt_middle], offset=0.0, is_init=False, move_style=RobotMoveStyle.move_joint_line)
            if ret is False:
                raise Exception("error")
            self.robot.move_offset(-0.30)

            self.move_joint([self.fs_ready_pose_joint])

            self.robot.tool_end = self.fs_pose[f"tool_end_{num}_pose"]
            self.current_switcher_num = self.get_curr_fs_num()

            return True

        except Exception as e:
            print(f"connect grasper failed , {e}")
            self.move_joint([self.fs_ready_pose_joint])
            return False


if __name__ == "__main__":
    robot = RobotNode(info_queue=multiprocessing.Queue(), error_queue=multiprocessing.Queue(),
                      tool_end=dict(pos=(0, 0, 0), ori=(0, 0, 0)), robot_brand="dobot",
                      glob_speed_ratio=50, payload=0.5)
    fs = FastSwitcher(str(Path(__file__).parent / "config/fs_pose.json"),
                      str(Path(__file__).parent / f"config/init_pose_joint.txt"),
                      robot)
    # fs.get_save_fs_pose_json()
    fs.connect_switcher(2)
