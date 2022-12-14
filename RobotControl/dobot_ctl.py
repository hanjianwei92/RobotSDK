import numpy as np
from scipy.spatial.transform import Rotation
from .dobot import *
from queue import Queue
from .coor_and_move_style import *


class DobotControl:
    def __init__(self, default_robot: bool = True,
                 logger=None,
                 tool_end=None,
                 ip='192.168.5.1',
                 port_ctl=29999, port_move=30003,
                 joint_max_vel=50, joint_max_acc=50,
                 line_max_vel=50, line_max_acc=50):
        """
        :param default_robot: 是否以默认参数初始化机械臂
        :param tool_end: 末端工具定义输入为字典dict(pos = (x,y,z),ori = (Rx, Ry, Rz)）
        :param ip: 机械臂IP
        :param port_ctl: 机械臂Port
        """
        self.robot_brand = 'dobot'
        if tool_end is None:
            tool_end = dict(pos=(0, 0, 0), ori=(0, 0, 0))

        if logger is None:
            logger = CasLogger("dobot.log", info_queue=Queue(), error_queue=Queue())

        self.log = logger
        self.robot_ctl = DobotApiDashboard(ip, port_ctl, logger)
        self.robot_move = DobotApiMove(ip, port_move, logger)

        # 用户工具描述
        self.tool_end = tool_end
        if default_robot is True:
            self.init_robot_by_default()
        self.set_joint_max_vel_and_acc(joint_max_vel=joint_max_vel, joint_max_acc=joint_max_acc)
        self.set_end_max_vel_and_acc(line_max_vel=line_max_vel, line_max_acc=line_max_acc)

    def init_robot_by_default(self, collision_level=5):
        """
        以默认参数初始化机械臂
        """
        if self.robot_ctl.EnableRobot()[0] != 0:
            self.log.error_show(f"EnableRobot() is Failed ")

        self.robot_ctl.User(0)
        self.robot_ctl.Tool(0)

        # 设置碰撞等级
        ret = self.robot_ctl.SetCollisionLevel(collision_level)[0]
        if ret != 0:
            self.log.error_show(f"RobotError 设置碰撞等级错误 错误代码为{ret}")
            return None

        if self.set_tool() is None:
            self.log.error_show("机械臂初始化错误")

    def set_tool(self,
                 payload=1.0,
                 inertia=0.1):
        """
        设置碰撞等级与工具动力学参数
        :param payload, # 负载（kg）
        :param inertia # 负载惯量 kgm²
        :return:
        """
        # 设置工具动力学参数
        ret = self.robot_ctl.PayLoad(payload, inertia)[0]
        if ret != 0:
            self.log.error_show(f"RobotError 设置工具动力学参数 错误代码为{ret}")
            return None
        return 1

    def set_joint_max_vel_and_acc(self, joint_max_vel=50, joint_max_acc=50):
        """
        设置机器人关节最大速度与加速度
        :param joint_max_vel: 单位比例（1-100）
        :param joint_max_acc:  单位比例（1-100）
        :return:
        """
        if self.robot_ctl.AccJ(joint_max_acc)[0]:
            self.log.error_show(f"设置关节加速度失败")

        if self.robot_ctl.SpeedJ(joint_max_vel)[0]:
            self.log.error_show(f"设置关节速度失败")

    def set_end_max_vel_and_acc(self, line_max_vel=0.45, line_max_acc=0.45):
        """
        设置机器人末端最大速度与加速度
        :param line_max_vel: 末端最大线速度  单位比例（1-100）
        :param line_max_acc: 末端最大线加速度 单位比例（1-100）

        :return:
        """
        if self.robot_ctl.AccL(line_max_acc)[0]:
            self.log.error_show(f"设置笛卡尔空间加速度失败")

        if self.robot_ctl.SpeedL(line_max_vel)[0]:
            self.log.error_show(f"设置笛卡尔空间速度失败")

    def get_current_waypoint(self):
        """
        得到当前法兰盘位姿（基座标系下）
        :return: 六个关节角 'joint': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0] °
                      位置 'pos': 位置[x, y, z] m
                      姿态 'ori': 姿态[Rx, Ry, Rz] °
        """
        error_id1, joint_angle = self.robot_ctl.GetAngle()
        error_id2, pos = self.robot_ctl.GetPose()
        if error_id1 == 0 or error_id2 == 0:
            joint = joint_angle
            flange_pose = pos[:3]
            flange_ori = pos[3:]
            return joint, flange_pose, flange_ori
        else:
            self.log.error_show(f"获取当前位姿失败，错误代码为{error_id1}、{error_id2}")
            raise Exception()

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
        :param pos: 位置[x,y,z]
        :param ori: 姿态[Rx, Ry, Rz]
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
            self.log.error_show("No waypoints")
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
                as_euler('xyz', degrees=True).squeeze().tolist()
            last_waypoint_joint = [0, 0, 1] + waypoints_joint[cnt]
            # 逆解到关节空间
            waypoint_joint = self.robot_ctl.InverseSolution(flange_pos_on_base + flange_ori_on_base +
                                                            last_waypoint_joint)[1]
            if waypoint_joint is None:
                self.log.error_show("逆解失败")
                self.robot_ctl.ClearError()
                return None

            waypoints_joint.append(waypoint_joint)
        return waypoints_joint[1:]

    def move_offset(self, offset, move_style: RobotMoveStyle = RobotMoveStyle.move_joint_line):
        """
        向目标法线方向移动偏移距离
        """
        final_target_in_flange = self.pos_to_rmatrix([0, 0, offset], list(self.tool_end['ori']))
        rt_final = self.end_to_base(final_target_in_flange)
        pos = rt_final[:3, 3].squeeze().tolist()
        ori = Rotation.from_matrix(rt_final[:3, :3]).as_euler('xyz', degrees=True).squeeze().tolist()
        final_target_joint = self.robot_ctl.InverseSolution(pos + ori)[1]
        if final_target_joint is None:
            self.log.error_show("逆解失败")
            self.robot_ctl.ClearError()
            return
        self.move_to_waypoints_in_joint([final_target_joint], move_style=move_style)

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
                    curr_joint = self.get_current_waypoint()[0]
                    for i in range(6):
                        if abs(curr_joint[i] - waypoint[i]) > check_joints_degree_range[i]:
                            self.log.error_show(f"机械臂第{i}轴移动{abs(curr_joint[i] - waypoint[i])}度，\
                                           所设的范围为{check_joints_degree_range[i]}度，超过移动范围")
                            raise Exception(f"机械臂第{i}轴移动{abs(curr_joint[i] - waypoint[i])}度，\
                                            所设的范围为{check_joints_degree_range[i]}度，超过移动范围")

                self.robot_move.JointMovJ(waypoint)
                self.robot_move.Sync()
                error_info = self.robot_ctl.GetErrorID()
                if error_info != "":
                    self.log.error_show("关节移动失败")
                    self.log.error_show(error_info)
                    self.robot_ctl.ClearError()
                    raise Exception(error_info)

        elif move_style is RobotMoveStyle.move_joint_line:
            for waypoint in waypoints_joint:
                waypoint_pos = self.robot_ctl.PositiveSolution(waypoint)[1]
                self.robot_move.MovL(waypoint_pos)
                self.robot_move.Sync()
                error_info = self.robot_ctl.GetErrorID()
                if error_info != "":
                    self.log.error_show("直线移动失败")
                    self.log.error_show(error_info)
                    self.robot_ctl.ClearError()
                    raise Exception(error_info)

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
        ret = self.move_to_waypoints_in_joint(waypoints_in_joint, move_style, check_joints_degree_range)
        if ret is None:
            self.log.error_show("移动失败")
            return
        if is_move_offset is True:
            self.move_offset(offset)

    def move_stop(self):
        """
        停止机器人运动
        :return:
        """
        self.robot_ctl.EmergencyStop()

    def get_current_robot_state(self):
        """
        得到当前机器人运行状态
        :return:0停止  1运行
        """
        return self.robot_ctl.RobotMode()[1]

    def disconnect_robot(self):
        self.robot_ctl.DisableRobot()


if __name__ == "__main__":
    robot = DobotControl()
    _, curr_pos, curr_ori = robot.get_current_waypoint()
    rt_o = robot.pos_to_rmatrix(curr_pos, curr_ori)
    pos = [-0.1044132, 0.773865, 0.11709, 176.05, -1.958, -22.7447]
    rt = robot.pos_to_rmatrix(pos[:3], pos[3:])
    print("开始移动")
    robot.move_to_waypoints([rt], is_move_offset=True, offset=0.15)
    print("移动完毕")
    robot.move_to_waypoints([rt_o])
    # robot.disconnect_robot()
