import time
from RobotControl import CasLogger, DobotControl, DobotApiState, RobotMoveStyle, RobotCoorStyle, \
               RMHandModbus, DhModbus, UniversalGraspHandCtl
from pathlib import Path
import multiprocessing
import threading
import numpy as np
from PyQt5.QtCore import QObject


def execute_grasp(command_move_queue,
                  command_ctl_queue,
                  command_ctl_result_queue,
                  result_value,
                  running_value,
                  info_queue,
                  error_queue,
                  tool_end,
                  robot_brand,
                  robot_init_joint,
                  state_array=None):
    log = CasLogger('logfiles/robot_move_log.log',
                    info_queue=info_queue,
                    error_queue=error_queue)
    try:
        if robot_brand == "aubo":
            import platform
            if platform.system() == "Windows" and platform.python_version().rsplit(".", 1)[0] != "3.7":
                log.error_show(f"aubo机器人在Windows下只支持python 3.7.x版本, 当前python版本为{platform.python_version()}")
                raise Exception(f"aubo机器人在Windows下只支持python 3.7.x版本, 当前python版本为{platform.python_version()}")
            from RobotControl.aubo_ctl import AuboControl
            robot_control = AuboControl(default_robot=True, tool_end=tool_end,
                                        max_line_vel=0.2, max_line_acc=0.1,
                                        max_angular_vel=40, max_angular_acc=20,
                                        max_joint_vel=90, max_joint_acc=50,
                                        info_queue=info_queue, error_queue=error_queue)
            running_value.value = -1
            log.finish_show(f"机械臂{robot_brand}初始化成功")

        elif robot_brand == "dobot":
            robot_control = DobotControl(default_robot=True,
                                         logger=log,
                                         tool_end=tool_end,
                                         global_speed=70, payload=2.5, collision_level=3,
                                         joint_max_vel=70, joint_max_acc=70,
                                         line_max_vel=50, line_max_acc=50,
                                         state_data_array=state_array)
            running_value.value = -1
            log.finish_show(f"机械臂{robot_brand}初始化成功")

        elif robot_brand == "moveit2":
            from RobotControl.moveit2_ctl import Moveit2Control
            robot_control = Moveit2Control(tool_end=tool_end,
                                           logger=log)
            running_value.value = -1
            log.finish_show(f"{robot_brand}初始化成功")

        else:
            robot_control = None
            log.error_show(f"未找到机器人{robot_brand}")

    except Exception as e:
        running_value.value = -2
        log.error_show(f"机械臂初始化失败{str(e)}")
        return

    grasp_init = False
    hand_ctl = None
    try:
        robot_init_pos = \
            tuple(np.loadtxt(robot_init_joint))
    except Exception as e:
        robot_init_pos = None
        log.error_show(f"未获得机器人初始位姿，{e}，设置为{robot_init_pos}")

    t = threading.Thread(target=cmd_ctl_thread, args=(robot_control, log, command_ctl_queue, command_ctl_result_queue))
    t.daemon = True
    t.start()

    while True:
        tmp = command_move_queue.get()
        # log.info_show("机械臂收到命令")
        pose_list, coor, move_style, offset, joint_list, grasp, grasp_pos, \
            grasp_force, suck, is_init_pose, check_joints_range, grasper_select = tmp

        if grasper_select != 0:
            try:
                if grasper_select == 1:
                    hand_ctl = DhModbus(connect_type=0, rtu_port_name='COM3', baud_rate=115200, log=log)
                elif grasper_select == 2:
                    hand_ctl = DhModbus(connect_type=1, ip="192.168.5.12", port=502, log=log)
                elif grasper_select == 3:
                    hand_ctl = DhModbus(connect_type=2, dobot_robot=robot_control, log=log)
                elif grasper_select == 4:
                    hand_ctl = RMHandModbus(connect_type=0, rtu_port_name='COM4', baud_rate=115200, log=log)
                elif grasper_select == 5:
                    hand_ctl = RMHandModbus(connect_type=1, ip="192.168.5.12", port=502, log=log)
                elif grasper_select == 6:
                    hand_ctl = RMHandModbus(connect_type=2, dobot_robot=robot_control, log=log)
                elif grasper_select == 7:
                    from RobotControl.moveit2_ctl import GripperMoveit
                    hand_ctl = GripperMoveit()
                else:
                    hand_ctl = UniversalGraspHandCtl(log=log)

                grasp_init = True
                log.finish_show("夹爪初始成功")

            except Exception as e:
                log.error_show(f"夹爪初始化失败{str(e)}")
                grasp_init = False
                hand_ctl = None

        if pose_list is not None:
            result_value.value = 0
            # log.info_show("机械臂执行位置移动")
            try:
                running_value.value = 1
                robot_control.move_to_waypoints(waypoints=pose_list,
                                                coor=coor,
                                                move_style=move_style,
                                                offset=offset,
                                                check_joints_degree_range=check_joints_range)
                result_value.value = 1
                if command_move_queue.empty() is True:
                    running_value.value = -1

            except Exception as e:
                log.error_show("机械臂位置移动失败，返回原点")
                log.error_show(str(e))
                try:
                    if is_init_pose and robot_init_pos is not None:
                        robot_control.move_to_waypoints_in_joint([robot_init_pos])
                except Exception as e:
                    log.error_show("机械臂移动至初始点失败")
                    log.error_show(str(e))
                result_value.value = -1
                if command_move_queue.empty() is True:
                    running_value.value = -1

        elif joint_list is not None:
            result_value.value = 0
            # log.info_show("机械臂开始关节运动")
            try:
                running_value.value = 1
                robot_control.move_to_waypoints_in_joint(waypoints_joint=joint_list,
                                                         move_style=move_style,
                                                         check_joints_degree_range=check_joints_range)
                result_value.value = 1
                if command_move_queue.empty() is True:
                    running_value.value = -1

            except Exception as e:
                log.error_show(f"机械臂关节移动失败{str(e)}，返回原点")
                try:
                    if is_init_pose and robot_init_pos is not None:
                        robot_control.move_to_waypoints_in_joint([robot_init_pos])
                except Exception as e:
                    log.error_show("机械臂移动失败")
                    log.error_show(str(e))
                result_value.value = -1
                if command_move_queue.empty() is True:
                    running_value.value = -1

        elif offset != 0:
            result_value.value = 0
            # log.info_show("机械臂偏移运动")
            try:
                running_value.value = 1
                robot_control.move_offset(offset=offset)
                result_value.value = 1
                if command_move_queue.empty() is True:
                    running_value.value = -1
            except Exception as e:
                log.error_show(f"机械臂偏移移动失败，返回原点,{e}")
                try:
                    if is_init_pose and robot_init_pos is not None:
                        robot_control.move_to_waypoints_in_joint([robot_init_pos])
                except Exception as e:
                    log.error_show("机械臂移动失败")
                    log.error_show(str(e))
                result_value.value = -1
                if command_move_queue.empty() is True:
                    running_value.value = -1

        elif grasp is not None and grasp_init is True:
            if grasp is True:
                hand_ctl.grasp(pos=0, force=grasp_force)
            else:
                hand_ctl.release(pos=grasp_pos)

        elif suck is not None and grasp_init is True:
            if suck is True:
                hand_ctl.suck()
            else:
                hand_ctl.release_suck()


def cmd_ctl_thread(robot_clt, log, command_ctl_queue, ctl_result_queue):
    while True:
        cmd_num, arg1, arg2, arg3, arg4 = command_ctl_queue.get()
        if cmd_num == 1:
            try:
                curr_way_point = robot_clt.get_current_waypoint()
                ctl_result_queue.put(curr_way_point)
            except Exception as e:
                log.error_show(f"获取当前坐标失败{str(e)}")
                ctl_result_queue.put(-1)

        elif cmd_num == 2:
            try:
                way_points = robot_clt.get_move_to_waypoints(arg1, arg2, arg3)
                ctl_result_queue.put(way_points)
            except Exception as e:
                log.error_show(f"获取目标位置逆解结果失败{str(e)}")
                ctl_result_queue.put(-1)

        elif cmd_num == 3:
            try:
                status = robot_clt.get_current_robot_state()
                ctl_result_queue.put(status)
            except Exception as e:
                log.error_show(f"获取机器人当前状态失败{e}")
                ctl_result_queue.put(-1)

        elif cmd_num == 4:
            try:
                status = robot_clt.move_stop()
                ctl_result_queue.put(status)
            except Exception as e:
                log.error_show(f"停止机器人失败{str(e)}")
                ctl_result_queue.put(-1)

        elif cmd_num == 5:
            try:
                status = robot_clt.disconnect_robot()
                ctl_result_queue.put(status)
            except Exception as e:
                log.error_show(f"关闭机器人失败{str(e)}")
                ctl_result_queue.put(-1)

        elif cmd_num == 6:
            try:
                rt_mat = robot_clt.pos_to_rmatrix(arg1, arg2)
                ctl_result_queue.put(rt_mat)
            except Exception as e:
                log.error_show(f"转换位姿矩阵失败{str(e)}")
                ctl_result_queue.put(-1)

        elif cmd_num == 7:
            try:
                end_pos = robot_clt.get_tool_end_pos()
                ctl_result_queue.put(end_pos)
            except Exception as e:
                log.error_show(f"获取末端在基坐标系下坐标失败{str(e)}")
                ctl_result_queue.put(-1)

        elif cmd_num == 8:
            try:
                ctl_result_queue.put(robot_clt.tool_end)
            except Exception as e:
                log.error_show(f"获得工具末端失败{str(e)}")
                ctl_result_queue.put(-1)
                
        elif cmd_num == 9:
            try:
                robot_clt.tool_end = arg1
                ctl_result_queue.put(0)
            except Exception as e:
                log.error_show(f"设置工具末端失败{str(e)}")
                ctl_result_queue.put(-1)

        elif cmd_num == 20 and robot_clt.robot_brand == "dobot":
            try:
                error_id = robot_clt.robot_ctl.ClearError()
                ctl_result_queue.put(error_id)
            except Exception as e:
                log.error_show(f"清除dobot错误失败{str(e)}")
                ctl_result_queue.put(-1)

        elif cmd_num == 21 and robot_clt.robot_brand == "dobot":
            try:
                error_id = robot_clt.robot_ctl.StartDrag(arg1)
                ctl_result_queue.put(error_id)
            except Exception as e:
                log.error_show(f"开启/关闭dobot机器人拖拽失败{str(e)}")
                ctl_result_queue.put(-1)


class RobotNode(QObject):
    def __init__(self,
                 info_queue,
                 error_queue,
                 tool_end,
                 robot_brand,
                 robot_init_joint=None):
        super().__init__()
        self.command_move_queue = multiprocessing.Queue(1)
        self.command_ctl_queue = multiprocessing.Queue(1)
        self.command_ctl_result_queue = multiprocessing.Queue(1)
        self.result_value = multiprocessing.Value('h', 0)  # 机械臂执行结果，0初始化，-1失败，1成功
        self.running_value = multiprocessing.Value('h', 0)  # 机械臂当前状态，0初始化，-1待机，-2初始化失败，1运行
        self.info_queue = info_queue
        self.error_queue = error_queue
        self.tool_end_pose = tool_end
        self.robot_brand = robot_brand
        self.robot_init_joint = robot_init_joint
        self.log = CasLogger('logfiles/robot_ctl_log.log',
                             info_queue=info_queue,
                             error_queue=error_queue)
        self.execute_grasp_async()
        while True:
            time.sleep(0.5)
            if self.running_value.value == -1:
                break
            elif self.running_value.value == -2:
                raise Exception(f"机械臂{robot_brand}初始化失败")
            else:
                continue

    def execute_grasp_async(self):
        if self.robot_brand == "dobot":
            robot_state = DobotApiState()
            grasp_process = multiprocessing.Process(target=execute_grasp, args=(self.command_move_queue,
                                                                                self.command_ctl_queue,
                                                                                self.command_ctl_result_queue,
                                                                                self.result_value,
                                                                                self.running_value,
                                                                                self.info_queue,
                                                                                self.error_queue,
                                                                                self.tool_end_pose,
                                                                                self.robot_brand,
                                                                                self.robot_init_joint,
                                                                                robot_state.data_array))

        else:
            grasp_process = multiprocessing.Process(target=execute_grasp, args=(self.command_move_queue,
                                                                                self.command_ctl_queue,
                                                                                self.command_ctl_result_queue,
                                                                                self.result_value,
                                                                                self.running_value,
                                                                                self.info_queue,
                                                                                self.error_queue,
                                                                                self.tool_end_pose,
                                                                                self.robot_brand,
                                                                                self.robot_init_joint))

        grasp_process.daemon = True
        grasp_process.start()

    def send_grasp_cmd(self, pose_list: list = None, coor=0, move_style=0, offset=0.0, joint_list: list = None,
                       grasp=None, grasp_pos=None, grasp_force=None, suck=None, is_init_pose=True,
                       check_joints_range: list = None, grasper_select=0):
        """
        Args:
            pose_list: 位姿矩阵（4x4）Rt的列表
            coor: 所设位姿矩阵的坐标系（可以是基座标系0，法兰盘坐标系1，工具末端为2）
            move_style: 移动插补方式（关节插补0，直线插补1）
            offset: 在目标点法线方向偏移的距离
            joint_list: 6个关节角度列表的列表（Nx6）
            grasp: 是否抓取
            grasp_pos: 夹爪张开幅度 （0-0.145, 0.145代表145mm）
            grasp_force: 夹爪抓取力（20-100%, 100代表105N）
            suck: 是否吸取
            is_init_pose:是否回到初始位姿
            check_joints_range: 6个关节角度的最大移动范围（角度）的列表
            grasper_select: 选择使用的末端夹具其中 1：大寰485，2：大寰TCP，3：大寰越疆
                                               4：增广485，5：增广TCP，6：增广越疆
                                               7： gripper_moveit2， 8 or others：通用气动夹爪吸盘

        Returns:
        """
        tmp = (pose_list, coor, move_style, offset, joint_list, grasp, grasp_pos, grasp_force, suck, is_init_pose,
               check_joints_range, grasper_select)
        self.command_move_queue.put(tmp)

    def send_ctl_cmd(self, cmd_num, arg1=None, arg2=None, arg3=None, arg4=None):
        """
        Args:
            cmd_num: 命令代码
                1：get_current_waypoint。2：get_move_to_waypoints。3：get_current_robot_state。
                4：move_stop。5：disconnect_robot。6：pos_to_rmatrix。7：get_tool_end_pos。
                8：get_tool。9：set_tool_end
                20：ClearError。 21：StartDrag
            arg1: 参数一
            arg2: 参数二
            arg3: 参数三
            arg4: 参数四
        """
        tmp = (cmd_num, arg1, arg2, arg3, arg4)
        self.command_ctl_queue.put(tmp)

    def move_pose(self, pose_list, coor=0, move_style=0, offset=0.0, is_init=True, check_joints_range=None):
        if check_joints_range is None:
            check_joints_range = [110, 110, 110, 110, 130, 180]
        self.result_value.value = 0
        self.send_grasp_cmd(pose_list=pose_list,
                            coor=coor,
                            move_style=move_style,
                            offset=offset,
                            is_init_pose=is_init,
                            check_joints_range=check_joints_range)
        while True:
            time.sleep(0.1)
            if self.result_value.value == 1:
                self.result_value.value = 0
                # self.log.finish_show("机械臂移动至目标点成功")
                return True
            elif self.result_value.value == -1:
                self.result_value.value = 0
                self.log.error_show("机械臂移动至目标点失败")
                return False
            else:
                continue

    def move_joint(self, joint_list, move_style=RobotMoveStyle.move_joint, check_joints_range=None):
        if check_joints_range is None:
            check_joints_range = [110, 110, 110, 110, 130, 360]
        self.result_value.value = 0
        self.send_grasp_cmd(joint_list=joint_list,
                            move_style=move_style,
                            check_joints_range=check_joints_range)
        while True:
            time.sleep(0.1)
            if self.result_value.value == 1:
                self.result_value.value = 0
                # self.log.finish_show("机械臂移动至目标点成功")
                return True
            elif self.result_value.value == -1:
                self.result_value.value = 0
                self.log.error_show("机械臂移动至目标点失败")
                return False
            else:
                continue

    def move_offset(self, offset):
        self.result_value.value = 0
        self.send_grasp_cmd(offset=offset)
        while True:
            time.sleep(0.1)
            if self.result_value.value == 1:
                self.result_value.value = 0
                # self.log.finish_show("机械臂移动至目标点成功")
                return True
            elif self.result_value.value == -1:
                self.result_value.value = 0
                self.log.error_show("机械臂移动至目标点失败")
                return False
            else:
                continue

    def grasper_execute(self, is_grasp, pos=None, force=None):
        self.send_grasp_cmd(grasp=is_grasp, grasp_pos=pos, grasp_force=force)

    def sucker_execute(self, is_suck):
        self.send_grasp_cmd(suck=is_suck)

    def switch_grasper(self, grasper):
        self.send_grasp_cmd(grasper_select=grasper)

    def get_current_waypoint(self):
        self.send_ctl_cmd(1)
        joint, end_pos, end_ori = self.command_ctl_result_queue.get()
        return joint, end_pos, end_ori

    def get_move_to_waypoints(self, waypoints: list = None, coor: RobotCoorStyle = RobotCoorStyle.base, offset=0.0):
        self.send_ctl_cmd(2, waypoints, coor, offset)
        waypoints_joint = self.command_ctl_result_queue.get()
        return waypoints_joint

    def get_current_robot_state(self):
        self.send_ctl_cmd(3)
        error_id = self.command_ctl_result_queue.get()
        return error_id

    def stop_move(self):
        self.send_ctl_cmd(4)
        error_id = self.command_ctl_result_queue.get()
        return error_id

    def disconnect_robot(self):
        self.send_ctl_cmd(5)
        error_id = self.command_ctl_result_queue.get()
        return error_id

    def pos_to_rmatrix(self, pos, ori):
        self.send_ctl_cmd(6, pos, ori)
        rt_mat = self.command_ctl_result_queue.get()
        return rt_mat

    def get_tool_end_pos(self):
        self.send_ctl_cmd(7)
        pos, ori = self.command_ctl_result_queue.get()
        return pos, ori

    @property
    def tool_end(self) ->dict:
        self.send_ctl_cmd(8)
        return self.command_ctl_result_queue.get()

    @tool_end.setter
    def tool_end(self, pose:dict):
        self.send_ctl_cmd(9, pose)
        self.command_ctl_result_queue.get()

    def start_drag(self, status):
        self.send_ctl_cmd(21, status)
        error_id = self.command_ctl_result_queue.get()
        return error_id

    def clear_error(self):
        self.send_ctl_cmd(20)
        error_id = self.command_ctl_result_queue.get()
        return error_id


if __name__ == "__main__":
    info_queue = multiprocessing.Queue()
    error_queue = multiprocessing.Queue()
    robot = RobotNode(info_queue, error_queue, tool_end=dict(pos=(0, 0.0, 0.52), ori=(0, 0, 0)),
                      robot_brand="moveit2")
    while True:
        robot.move_joint([(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)])
        time.sleep(5)
