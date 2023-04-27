import time
import numpy as np
import sys
import multiprocessing
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from threading import Thread
from geometry_msgs.msg import Point, Quaternion
from .pymoveit2 import MoveIt2, GripperCommand
from .pymoveit2.robots import casia_robot
from .coor_and_move_style import *
from .dobot.cas_logger import CasLogger


class Moveit2Control:
    def __init__(self,
                 tool_end=None,
                 logger=None,
                 joint_max_vel=50, joint_max_acc=50,
                 line_max_ve0l=50, line_max_acc=50):

        self.robot_brand = 'moveit2'
        if tool_end is None:
            tool_end = dict(pos=(0, 0, 0), ori=(0, 0, 0))

        if logger is None:
            logger = CasLogger("dobot.log",
                               info_queue=multiprocessing.Queue(1),
                               error_queue=multiprocessing.Queue(1))

        # self.moveit_process = subprocess.Popen(['ros2', 'launch', 'casia_robot', 'demo.launch.py'])

        self.log: CasLogger = logger
        self.moveit_ctl: MoveIt2 = None
        self.tool_end: dict = tool_end
        self.init_moveit()

    def init_moveit(self):
        rclpy.init(args=sys.argv)
        node = Node("moveit_group")
        self.moveit_ctl = MoveIt2(
            node=node,
            joint_names=casia_robot.joint_names(),
            base_link_name=casia_robot.base_link_name(),
            end_effector_name=casia_robot.end_effector_name(),
            group_name=casia_robot.MOVE_GROUP_ARM,
            execute_via_moveit=False,
            ignore_new_calls_while_executing=True,
            callback_group=ReentrantCallbackGroup(),
            follow_joint_trajectory_action_name='/arm_group_controller/follow_joint_trajectory'
        )

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

    def close_moveit(self):
        # self.moveit_process.kill()
        rclpy.shutdown()
        exit(0)

    def get_current_waypoint(self):
        """
        得到当前法兰盘位姿（基座标系下）
        :return: 六个关节角 'joint': [180,50,50,10,260,560] °
                      位置 'pos': 位置[x, y, z] m
                      姿态 'ori': 姿态[Rx, Ry, Rz] °
        """
        joints = self.moveit_ctl.get_current_joints()
        joints = np.array(joints) / np.pi * 180.0
        pos, ori = self.moveit_ctl.get_current_pose("Link6")
        rpy = Rotation.from_quat(ori).as_euler('xyz', degrees=True)
        return joints.tolist(), pos, rpy

    def get_tool_end_pos(self):
        """
        得到工具末端在基坐标系下的坐标
        :return: pos,ori
        """
        _, flange_pose, flange_ori = self.get_current_waypoint()
        # 得到法兰盘工具末端点相对于基座坐标系中的位置
        curr_pos_mat = self.pos_to_rmatrix(flange_pose, flange_ori)
        tool_pos_mat_on_flange = self.pos_to_rmatrix(list(self.tool_end['pos']), list(self.tool_end['ori']))
        tool_pos_mat_on_base = curr_pos_mat @ tool_pos_mat_on_flange

        pos = tool_pos_mat_on_base[:3, 3:]
        ori = Rotation.from_matrix(tool_pos_mat_on_base[:3, :3]).as_euler('xyz', degrees=True)
        # self.log_info(f"工具末端当前位姿（在基坐标系）是{pos},姿态是{ori}")
        return pos, ori

    def end_to_base(self,
                    waypoints_end_rt_mat,
                    curr_pos=None,
                    curr_ori=None):
        """
        如果使用默认值，则讲法兰坐标系下的点转换值基座标系下
        :param waypoints_end_rt_mat: 特定坐标系下目标的rt矩阵
        :param curr_pos: 特定坐标系在基座标系下的位置[x, y, z] m
        :param curr_ori: 特定坐标系在基座标系下姿态[Rx, Ry, Rz] °
        :return: 基坐标系下，目标位姿矩阵
        """
        if curr_pos is None and curr_ori is None:
            _, curr_pos, curr_ori = self.get_current_waypoint()
        rt_end_to_base = self.pos_to_rmatrix(curr_pos, curr_ori)
        rt_waypoints_base = rt_end_to_base.dot(np.array(waypoints_end_rt_mat))
        return rt_waypoints_base

    @staticmethod
    def pos_to_rmatrix(pos: list, ori: list):
        """
        位姿转换为旋转平移矩阵
        :param pos: 位置[x,y,z] m
        :param ori: 姿态[Rx, Ry, Rz] °
        :return: RT旋转平移矩阵
        """
        rt_mat = np.eye(4)
        r_matrix = Rotation.from_euler('xyz', ori, degrees=True).as_matrix()
        rt_mat[:3, :3] = r_matrix
        rt_mat[:3, 3:] = np.reshape(pos, (3, 1))
        return rt_mat

    def get_move_to_waypoints(self, waypoints: list = None, coor: RobotCoorStyle = RobotCoorStyle.base, offset=0.0):
        """
        转换特定坐标系下的目标点位机器人关节运动坐标
        :param waypoints: 特定坐标系下的目标点的Rt矩阵列表
        :param coor: 目标点的坐标系定义，为base,flanger,tool_end
        :param offset: 偏移距离
        :return: 关节空间下的目标点坐标
        """
        if waypoints is None:
            return None
        waypoints_joint = []
        # 当前机器人坐标
        flange_in_base = self.get_current_waypoint()
        tool_end_in_base = self.get_tool_end_pos()
        current_waypoint_joint = flange_in_base[0]
        waypoints_joint.append(current_waypoint_joint)
        if coor == RobotCoorStyle.tool_end:
            curr_pos, curr_ori = tool_end_in_base
        else:
            curr_pos = flange_in_base[1]
            curr_ori = flange_in_base[2]

        for cnt in range(len(waypoints)):
            if coor == RobotCoorStyle.base:
                tool_waypoint_base = waypoints[cnt]
            elif coor == RobotCoorStyle.tool_end:
                tool_waypoint_base = self.end_to_base(waypoints[cnt], curr_pos, curr_ori)
            else:
                tool_waypoint_base = self.end_to_base(waypoints[cnt])

            if tool_waypoint_base is None:
                self.log.error_show("转换到基座标系失败")
                return None

            flange_pos_on_end = np.array(
                [-self.tool_end['pos'][0], -self.tool_end['pos'][1], -(self.tool_end['pos'][2] + offset),
                 1]).reshape((4, 1))
            flange_pos_on_base = np.dot(tool_waypoint_base, flange_pos_on_end)[:3].squeeze().tolist()
            flange_ori_on_base = Rotation.from_matrix(tool_waypoint_base[:3, :3]). \
                as_quat().squeeze().tolist()
            last_waypoint_joint = np.array(waypoints_joint[cnt]) / 180.0 * np.pi
            # 逆解到关节空间
            waypoint_joint = self.moveit_ctl.get_ik_result(flange_pos_on_base, flange_ori_on_base, "Link6",
                                                           last_waypoint_joint.tolist())

            if waypoint_joint is None:
                self.log.error_show("逆解失败")
                return None

            waypoint_joint = np.array(waypoint_joint) / np.pi * 180.0
            waypoints_joint.append(waypoint_joint.tolist())
        return waypoints_joint[1:]

    def move_offset(self, offset, move_style: RobotMoveStyle = RobotMoveStyle.move_joint_line):
        """
        向目标法线方向移动偏移距离
        """
        final_target_in_flange = self.pos_to_rmatrix([0, 0, offset], list(self.tool_end['ori']))
        rt_final = self.end_to_base(final_target_in_flange)
        pos = rt_final[:3, 3].squeeze().tolist()
        ori = Rotation.from_matrix(rt_final[:3, :3]).as_quat().squeeze().tolist()
        curr_point_joint = np.array(self.get_current_waypoint()[0]) / 180.0 * np.pi
        final_target_joint = self.moveit_ctl.get_ik_result(pos, ori, "Link6", curr_point_joint.tolist())
        if final_target_joint is None:
            self.log.error_show("move_offset, 逆解失败")
            raise Exception("move offset, 逆解失败")
        final_target_joint = np.array(final_target_joint) / np.pi * 180.0
        self.move_to_waypoints_in_joint([final_target_joint.tolist()], move_style=move_style)

    def move_to_waypoints_in_joint(self,
                                   waypoints_joint: list,
                                   move_style: RobotMoveStyle = RobotMoveStyle.move_joint,
                                   check_joints_degree_range: list = None):
        """
        :param waypoints_joint: 关节空间的路径点列表
        :param move_style: 移动类型
        :param check_joints_degree_range: 检查逆解轴的角度是否超出设定范围
        :return:
        """
        if move_style is RobotMoveStyle.move_joint:
            for waypoint in waypoints_joint:
                if check_joints_degree_range is not None:
                    check_joints_degree_range = np.array(check_joints_degree_range)
                    curr_joint = self.get_current_waypoint()[0]
                    for i in range(6):
                        if abs(curr_joint[i] - waypoint[i]) > check_joints_degree_range[i]:
                            self.log.error_show(f"机械臂第{i}轴移动{abs(curr_joint[i] - waypoint[i])}度，\
                                           所设的范围为{check_joints_degree_range[i]}度，超过移动范围")
                            raise Exception(f"机械臂第{i}轴移动{abs(curr_joint[i] - waypoint[i])}度，\
                                            所设的范围为{check_joints_degree_range[i]}度，超过移动范围")
                waypoint = np.array(waypoint) / 180.0 * np.pi
                self.moveit_ctl.move_to_joint(waypoint.tolist())
                res = self.moveit_ctl.wait_until_executed()
                if res != 1:
                    self.log.error_show(f"关节移动失败{res}")
                    raise Exception(f"关节移动失败{res}")

        elif move_style is RobotMoveStyle.move_joint_line:
            for waypoint in waypoints_joint:
                waypoint = np.array(waypoint) / 180.0 * np.pi
                self.moveit_ctl.move_to_joint(waypoint.tolist())
                res = self.moveit_ctl.wait_until_executed()
                if res != 1:
                    self.log.error_show(f"直线移动失败{res}")
                    raise Exception(f"直线移动失败{res}")

        return 1

    def move_to_waypoints(self,
                          waypoints: list,
                          coor: RobotCoorStyle = RobotCoorStyle.base,
                          move_style: RobotMoveStyle = RobotMoveStyle.move_joint,
                          offset: float = 0.0,
                          is_move_offset: bool = False,
                          check_joints_degree_range: list = None):
        """
        在法兰坐标系下移动机械臂到指定点
        :param waypoints: 指定坐标系下目标点的坐标，元素为目标点Rt)
        :param coor: 指定目标点的坐标系
        :param move_style: 移动类型
        :param offset: 机器人距离目标点的偏移
        :param is_move_offset: 是否需要偏移
        :param check_joints_degree_range: 检查逆解轴的角度是否超出设定范围
        :return:
        """
        waypoints_in_joint = self.get_move_to_waypoints(waypoints=waypoints, coor=coor, offset=offset)
        if waypoints_in_joint is None:
            self.log.error_show("move_to_waypoints, 逆解失败")
            raise Exception("move_to_waypoints, 逆解失败")
        self.move_to_waypoints_in_joint(waypoints_in_joint, move_style, check_joints_degree_range)
        if is_move_offset is True:
            self.move_offset(offset)

    def get_current_robot_state(self):
        """
        得到当前机器人运行状态
        :return:0停止  1运行
        """
        return self.moveit_ctl.get_robot_state()

    def move_stop(self):
        """
        停止机器人运动
        :return:
        """
        return self.moveit_ctl.cancel_current_motion()

    def disconnect_robot(self):
        pass

    def start_drag(self):
        pass

    def clear_error(self):
        pass


class GripperMoveit:
    def __init__(self, init_rclpy=False):
        self.gripper_moveit = None
        self.init_gripper_moveit(init_rclpy)

    def init_gripper_moveit(self, init_rclpy=False):
        if init_rclpy:
            rclpy.init(args=sys.argv)
        node = Node("gripper_moveit_group")
        self.gripper_moveit = GripperCommand(
            node=node,
            gripper_joint_names=['dh_gripper_finger1_joint'],
            open_gripper_joint_positions=[-37.0],
            closed_gripper_joint_positions=[0.0],
            gripper_command_action_name='/hand_group_controller/gripper_cmd'
        )

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

    def get_current_position(self):
        return abs(self.gripper_moveit.get_gripper_joint_position[0]) / np.pi * 180.0 / 37.0 * 0.145

    def grasp(self, pos=0.0, force=40.0):
        pos = 0.0 if pos is None else pos
        force = 60.0 if force is None else force

        self.gripper_moveit.gripper_go(-(pos / 0.145 * 37.0 / 180.0 * np.pi), float(force))

    def release(self, pos=0.145):
        pos = 0.145 if pos is None else pos
        self.gripper_moveit.gripper_go(-(pos / 0.145 * 37.0 / 180.0 * np.pi), 100.0)


if __name__ == "__main__":
    moveit = Moveit2Control()
    while True:
        try:
            moveit.move_to_waypoints_in_joint([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
            print(f"{moveit.get_current_waypoint()}")
            time.sleep(5)
        except Exception as e:
            print(f"{e} moveit.close_moveit()")

    # gripper = GripperMoveit()
    # while True:
    #     key = input()
    #     if key == " ":
    #         gripper.grasp(0.145, 100.0)
    #     else:
    #         gripper.grasp(0.0, 100.0)
