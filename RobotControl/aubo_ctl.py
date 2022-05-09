import math
import numpy as np
from scipy.spatial.transform import Rotation
from queue import Queue
from .coor_and_move_style import *
import datetime
import platform
if platform.system() == "Windows":
    from .aubo_windows import *
else:
    from .aubo_linux import *


class AuboControl:
    def __init__(self, default_robot: bool,
                 max_line_vel=0.3, max_line_acc=0.3,
                 max_angular_vel=50, max_angular_acc=25,
                 max_joint_vel=90, max_joint_acc=50,
                 tool_end=None,
                 ip='192.168.1.100', port=8899,
                 info_queue=None, error_queue=None):
        """
        :param default_robot: 是否以默认参数初始化机械臂
        :param tool_end: 末端工具定义输入为字典dict(pos = (x,y,z),ori = (w, x, y, z)）
        :param ip: 机械臂IP
        :param port: 机械臂Port
        """
        self.robot_brand = 'aubo'
        if tool_end is None:
            tool_end = dict(pos=(0, 0, 0.296), ori=(1, 0, 0, 0))

        if info_queue is None or error_queue is None:
            self.info_queue = Queue()
            self.error_queue = Queue()
        else:
            self.info_queue = info_queue
            self.error_queue = error_queue

        logger_init()
        # 系统初始化
        Auboi5Robot.initialize()
        self.ip = ip
        self.port = port
        # 遨博机器人SDK实例
        self.robot = Auboi5Robot()
        # 机器人控制上下文
        self.handle = self.robot.create_context()
        self.log_info(f"机械臂控制句柄为rhsd is {self.handle}")
        # 用户工具描述
        self.tool_end = tool_end
        if default_robot is True:
            self.init_robot_by_default()
        self.set_joint_max_vel_and_acc(joint_max_vel=max_joint_vel, joint_max_acc=max_joint_acc)
        self.set_end_max_vel_and_acc(line_max_vel=max_line_vel, line_max_acc=max_line_acc,
                                     angular_max_acc=max_angular_acc, angular_max_vel=max_angular_vel)

    def log_info(self, info):
        logger.info(info)
        cur_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-4]
        self.info_queue.put("<font color=\"#000000\">" + str(cur_time) + ": " + info + "</font>")

    def log_error(self, error):
        logger.error(error)
        cur_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-4]
        self.error_queue.put("<font color=\"#FF0000\">" + "【ERROR】" + str(cur_time) + ": " + error + "</font>")

    def init_robot_by_default(self):
        """
        以默认参数初始化机械臂
        """
        if self.init_and_robot() is None:
            self.log_error("机械臂初始化错误")

        if self.set_tool() is None:
            self.log_error("机械臂初始化错误")

    def init_and_robot(self):
        # 连接机器人
        ret = self.robot.connect(self.ip, self.port)
        if ret != RobotErrorType.RobotError_SUCC:
            self.log_error(f"connect sever {self.ip} : {self.port} failed , ret is {ret}")
            return None
        # 关闭可能运行的机器人
        # self.robot.robot_shutdown()
        # 上电
        self.robot.robot_startup()
        # 初始化全局控制参数
        ret = self.robot.init_profile()
        if ret != RobotErrorType.RobotError_SUCC:
            self.log_error(f"RobotError 全局初始化错误 错误代码为{ret}")
            return None
        return 1

    def set_tool(self,
                 collision_class=8,
                 tool_dynamics=None):
        """
        设置碰撞等级与工具动力学参数
        :param collision_class: 碰撞等级
        :param tool_dynamics: 工具动力学参数（相对于法兰盘）
               tool_dynamics = dict(
                # 位置（米）
                position=(0.0, 0.0, 0.2),
                # 负载（kg）
                payload=1.0,
                # 惯量
                inertia=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        :return:
        """
        # 相对于法兰盘的工具动力学参数
        if tool_dynamics is None:
            tool_dynamics = dict(
                # 位置（米）
                position=(0.0, 0.0, 0.2),
                # 负载（kg）
                payload=1.5,
                # 惯量
                inertia=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        # 设置碰撞等级
        ret = self.robot.set_collision_class(collision_class)
        if ret != collision_class:
            self.log_error(f"RobotError 设置碰撞等级错误 错误代码为{ret}")
            return None
        # 设置工具动力学参数
        ret = self.robot.set_tool_dynamics_param(tool_dynamics)
        if ret != RobotErrorType.RobotError_SUCC:
            self.log_error(f"RobotError 设置工具动力学参数 错误代码为{ret}")
            return None
        # 设置工具运动学参数
        ret = self.robot.set_tool_kinematics_param(self.tool_end)
        if ret != RobotErrorType.RobotError_SUCC:
            self.log_error(f"RobotError 设置碰工具运动学参数 错误代码为{ret}")
            return None
        # # 设置工具末端参数
        ret = self.robot.set_tool_end_param(self.tool_end)
        if ret != RobotErrorType.RobotError_SUCC:
            self.log_error(f"RobotError 设置碰工具末端参数 错误代码为{ret}")
            return None
        return 1

    def set_joint_max_vel_and_acc(self, joint_max_vel=90, joint_max_acc=50):
        """
        设置机器人关节最大速度与加速度
        :param joint_max_vel: 单位 °/s
        :param joint_max_acc:  单位 °/s^2
        :return:
        """
        _joint_max_vel = [joint_max_vel / 180.0 * math.pi] * 6
        _joint_max_acc = [joint_max_acc / 180.0 * math.pi] * 6
        self.robot.set_joint_maxvelc(tuple(_joint_max_vel))
        self.robot.set_joint_maxacc(tuple(_joint_max_acc))

    def set_end_max_vel_and_acc(self,
                                line_max_vel=0.45, line_max_acc=0.45,
                                angular_max_vel=50, angular_max_acc=25):
        """
        设置机器人末端最大速度与加速度
        :param line_max_vel: 末端最大线速度  m/s
        :param line_max_acc: 末端最大线加速度 m/s^2
        :param angular_max_vel: 末端最大角速度 °/s
        :param angular_max_acc: 末端最大角加速度 °/s^2
        :return:
        """
        _angular_max_vel = angular_max_vel / 180.0 * math.pi
        _angular_max_acc = angular_max_acc / 180.0 * math.pi
        self.robot.set_end_max_line_velc(line_max_vel)
        self.robot.set_end_max_line_acc(line_max_acc)
        self.robot.set_end_max_angle_velc(_angular_max_vel)
        self.robot.set_end_max_line_acc(_angular_max_acc)

    def get_current_waypoint(self):
        """
        得到当前法兰盘位姿（基座标系下）
        :return: 六个关节角 'joint': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
                      位置 'pos': [-0.06403157614989634, -0.4185973810159096, 0.816883228463401]
                      姿态 'ori': [-0.11863209307193756, 0.3820514380931854, 0.0, 0.9164950251579285]
        """
        waypoint = self.robot.get_current_waypoint()
        if waypoint is None:
            self.log_error("获取当前位姿错误")
            return None
        joint = waypoint['joint']
        end_pos = waypoint['pos']
        end_ori = waypoint['ori']
        return joint, end_pos, end_ori

    def get_tool_end_pos(self):
        """
        得到工具末端在基坐标系下的坐标
        :return: list(tool_pos_on_base['pos']), list(tool_pos_on_base['ori'])
        """
        current_pos = self.robot.get_current_waypoint()
        tool_desc = self.tool_end
        # 得到法兰盘工具末端点相对于基座坐标系中的位置
        tool_pos_on_base = self.robot.base_to_base_additional_tool(current_pos['pos'],
                                                                   current_pos['ori'],
                                                                   tool_desc)
        pos = list(tool_pos_on_base['pos'])
        ori = list(tool_pos_on_base['ori'])
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
        :param curr_ori: 特定坐标系在基座标系下姿态[w, x, y, z]
        :return: 基坐标系下，目标位姿矩阵
        """
        if curr_pos is None and curr_ori is None:
            _, curr_pos, curr_ori = self.get_current_waypoint()
        rt_end_to_base = AuboControl.pos_to_rmatrix(curr_pos, curr_ori)
        rt_waypoints_base = rt_end_to_base.dot(np.array(waypoints_end_rt_mat))
        return rt_waypoints_base

    @staticmethod
    def pos_to_rmatrix(pos: list, ori: list):
        """
        位姿转换为旋转平移矩阵
        :param pos: 位置[x,y,z]
        :param ori: 姿态四元数[w, x, y, z]
        :return: RT旋转平移矩阵
        """
        rt_mat = np.eye(4)
        r_matrix = Rotation.from_quat([ori[1], ori[2], ori[3], ori[0]]).as_matrix()
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
            self.log_error("No waypoints")
            return None

        waypoints_joint = []
        # 当前机器人坐标
        flanger_in_base = self.get_current_waypoint()
        tool_end_in_end = self.get_tool_end_pos()
        current_waypoint_joint = flanger_in_base[0]
        waypoints_joint.append(current_waypoint_joint)
        if coor == RobotCoorStyle.tool_end:
            curr_pos, curr_ori = tool_end_in_end
        else:
            curr_pos = flanger_in_base[1]
            curr_ori = flanger_in_base[2]

        for cnt in range(len(waypoints)):
            if coor == RobotCoorStyle.base:
                tool_waypoint_base = waypoints[cnt]
            elif coor == RobotCoorStyle.tool_end:
                tool_waypoint_base = self.end_to_base(waypoints[cnt], curr_pos, curr_ori)
            else:
                tool_waypoint_base = self.end_to_base(waypoints[cnt])

            if tool_waypoint_base is None:
                self.log_error("转换到基座标系失败")
                return None

            flanker_pos_on_end = np.array(
                [-self.tool_end['pos'][0], -self.tool_end['pos'][1], -(self.tool_end['pos'][2] + offset),
                 1]).reshape((4, 1))
            flanker_pos_on_base = np.dot(tool_waypoint_base, flanker_pos_on_end)[:3]

            quat_flanker_on_base = Rotation.from_matrix(tool_waypoint_base[:3, :3]).as_quat()
            quat_flanker_on_base = [quat_flanker_on_base[3], quat_flanker_on_base[0], quat_flanker_on_base[1],
                                    quat_flanker_on_base[2]]

            # 逆解到关节空间
            waypoint_joint = self.robot.inverse_kin(waypoints_joint[cnt],
                                                    flanker_pos_on_base,
                                                    quat_flanker_on_base)
            if waypoint_joint is None:
                self.log_error("逆解失败")
                return None

            waypoints_joint.append(waypoint_joint['joint'])
        return waypoints_joint[1:]

    def move_offset(self, offset, move_style: RobotMoveStyle = RobotMoveStyle.move_joint_line):
        """
        向目标法线方向移动偏移距离
        """
        final_target_in_flanger = self.pos_to_rmatrix([0, 0, offset], list(self.tool_end['ori']))
        curr_point_joint = self.get_current_waypoint()[0]
        rt_final = self.end_to_base(final_target_in_flanger)
        final_pos = rt_final[:3, 3]
        ori = Rotation.from_matrix(rt_final[:3, :3]).as_quat()
        final_ori = [ori[3], ori[0], ori[1], ori[2]]
        final_target_joint = self.robot.inverse_kin(curr_point_joint, final_pos, final_ori)
        self.move_to_waypoints_in_joint([final_target_joint['joint']], move_style=move_style)

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
        self.robot.remove_all_waypoint()
        if move_style is RobotMoveStyle.move_joint:
            for waypoint in waypoints_joint:
                if check_joints_degree_range is not None:
                    curr_joint = self.get_current_waypoint()[0]
                    for i in range(6):
                        if abs(curr_joint[i] - waypoint[i]) / np.pi * 180 > check_joints_degree_range[i]:
                            self.log_error(f"机械臂第{i}轴移动{abs(curr_joint[i] - waypoint[i]) / np.pi * 180}度，\
                                           所设的范围为{check_joints_degree_range[i]}度，超过移动范围")
                            raise Exception(f"机械臂第{i}轴移动{abs(curr_joint[i] - waypoint[i]) / np.pi * 180}度，\
                                            所设的范围为{check_joints_degree_range[i]}度，超过移动范围")

                ret = self.robot.move_joint(waypoint)
                if ret is not RobotErrorType.RobotError_SUCC:
                    self.log_error("关节移动失败")
                    return None

        elif move_style is RobotMoveStyle.move_joint_line:
            for waypoint in waypoints_joint:
                ret = self.robot.move_line(waypoint)
                if ret is not RobotErrorType.RobotError_SUCC:
                    self.log_error("关节移动失败")
                    return None
        return 1

    def move_to_waypoints(self,
                          waypoints: list = None,
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
        ret = self.move_to_waypoints_in_joint(waypoints_in_joint, move_style, check_joints_degree_range)
        if ret is None:
            self.log_error("移动失败")
            return
        if is_move_offset is True:
            self.move_offset(offset)

    def move_stop(self):
        """
        停止机器人运动
        :return:
        """
        self.robot.move_stop()

    def get_current_robot_state(self):
        """
        得到当前机器人运行状态
        :return:0停止  1运行
        """
        return self.robot.get_robot_state()

    def disconnect_robot(self):
        # self.robot.robot_shutdown()
        self.robot.disconnect()
        Auboi5Robot.uninitialize()
