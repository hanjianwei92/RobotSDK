import json
import zmq
import threading
import sys
import multiprocessing
from pathlib import Path
from typing import Optional
from RobotControl import DobotControl, DobotApiState, UniversalGraspHandCtl, DhModbus, RMHandModbus, \
    CasLogger, FastSwitcher


class RobotSeverZMQ:
    def __init__(self, port: int = 8100, log: CasLogger = None, recv_timeout: int = 10000):
        context = zmq.Context()
        # context.setsockopt(zmq.RCVTIMEO, recv_timeout)
        # context.setsockopt(zmq.SNDTIMEO, 1000)
        context.setsockopt(zmq.LINGER, 0)

        self.socket = context.socket(zmq.REP)
        self.socket.bind(f"tcp://*:{port}")

        if log is None:
            log = CasLogger("ZMQ_log.log")
        self.log = log
        self.robot_msg_dict = None

    def recv_msg_dict(self) -> Optional[dict]:
        try:
            self.robot_msg_dict = json.loads(self.socket.recv())
            return self.robot_msg_dict
        except Exception as e:
            self.log.error_show(f"recv_msg_failed, because of {e}")

    def send_recv_response(self, response: dict):
        try:
            self.socket.send_string(json.dumps(response))
        except Exception as e:
            self.log.error_show(f"send_msg_failed, because of {e}")

    def is_exist_in_dict(self, key) -> bool:
        return self.robot_msg_dict is not None and key in self.robot_msg_dict


def robot_cmd(robot_control, log):
    zmq_sever = RobotSeverZMQ(port=8101)
    while True:
        zmq_sever.recv_msg_dict()
        if zmq_sever.is_exist_in_dict("stop_move"):
            try:
                robot_control.move_stop()
                zmq_sever.send_recv_response({"stop_move": True})
            except Exception as e:
                log.error_show(f"机械臂停止失败{str(e)}")
                zmq_sever.send_recv_response({"stop_move": False})
            continue

        if zmq_sever.is_exist_in_dict("get_current_waypoint"):
            try:
                joint, end_pos, end_ori = robot_control.get_current_waypoint()
                zmq_sever.send_recv_response({"joint": joint,
                                              "end_pos": end_pos,
                                              "end_ori": end_ori})
            except Exception as e:
                log.error_show(f"获取当前坐标失败{str(e)}")
                zmq_sever.send_recv_response({"joint": [],
                                              "end_pos": [],
                                              "end_ori": []})
            continue

        if zmq_sever.is_exist_in_dict("get_inverse_solution"):
            param = zmq_sever.robot_msg_dict["get_inverse_solution"]
            try:
                pose_rt = robot_control.pos_to_rmatrix(param["pos"], param["ori"])
                joints_list = robot_control.get_move_to_waypoints(waypoints=[pose_rt],
                                                                  coor=param["coor"],
                                                                  offset=param["offset"])
                zmq_sever.send_recv_response({"joints": list(joints_list[0])})
            except Exception as e:
                log.error_show(f"获取目标位置逆解结果失败{str(e)}")
                zmq_sever.send_recv_response({"joints": []})
            continue

        if zmq_sever.is_exist_in_dict("disconnect_robot"):
            try:
                robot_control.disconnect_robot()
                zmq_sever.send_recv_response({"disconnect_robot": True})
            except Exception as e:
                log.error_show(f"关闭机器人失败{str(e)}")
                zmq_sever.send_recv_response({"disconnect_robot": False})
            finally:
                break

        if zmq_sever.is_exist_in_dict("pos_to_rmatrix"):
            param = zmq_sever.robot_msg_dict["pos_to_rmatrix"]
            try:
                rt_mat = robot_control.pos_to_rmatrix(pos=param["pos"], ori=param["ori"])
                zmq_sever.send_recv_response({"pos_to_rmatrix": list(rt_mat)})
            except Exception as e:
                log.error_show(f"pos_to_rmatrix失败{str(e)}")
                zmq_sever.send_recv_response({"pos_to_rmatrix": []})
            continue

        if zmq_sever.is_exist_in_dict("get_tool_end_pose"):
            try:
                pos, ori = robot_control.get_tool_end_pos()
                zmq_sever.send_recv_response({"pos": pos.squeeze().tolist(),
                                              "ori": ori.squeeze().tolist()})
            except Exception as e:
                log.error_show(f"获取末端在基坐标系下坐标失败{str(e)}")
                zmq_sever.send_recv_response({"pos": [],
                                              "ori": []})
            continue

        if zmq_sever.is_exist_in_dict("ClearError") and robot_control.robot_brand == "dobot":
            try:
                error_id = robot_control.robot_ctl.ClearError()
                zmq_sever.send_recv_response({"ClearError": True})
            except Exception as e:
                log.error_show(f"清除dobot错误失败{str(e)}")
                zmq_sever.send_recv_response({"ClearError": False})
            continue

        if zmq_sever.is_exist_in_dict("StartDrag") and robot_control.robot_brand == "dobot":
            param = zmq_sever.robot_msg_dict["StartDrag"]
            try:
                error_id = robot_control.robot_ctl.StartDrag(status=param["status"])
                zmq_sever.send_recv_response({"StartDrag": True})
            except Exception as e:
                log.error_show(f"开启/关闭dobot机器人拖拽失败{str(e)}")
                zmq_sever.send_recv_response({"StartDrag": False})
            continue


def robot_state_feedback(running_value: multiprocessing.Value,
                         result_value: multiprocessing.Value):
    context = zmq.Context()
    context.setsockopt(zmq.LINGER, 0)
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://*:8102")

    while True:
        running_state_value = running_value.get()
        result_state_value = result_value.get()
        socket.send_string(json.dumps({"running_value": running_state_value,
                                       "result_value": result_state_value}))


def zmq_sever_process(sys_argv: list):
    if len(sys_argv) < 2:
        robot_brand = "dobot"
    else:
        robot_brand = sys_argv[1]
    result_value = multiprocessing.Value('h', 0)
    running_value = multiprocessing.Value('h', 0)
    grasp_init = False
    hand_ctl = None

    log = CasLogger('logfiles/robot_move_log.log')
    zmq_sever = RobotSeverZMQ(port=8100, log=log)

    robot_state_process = multiprocessing.Process(target=robot_state_feedback,
                                                  args=(running_value, result_value),
                                                  daemon=True)
    robot_state_process.start()

    try:
        if robot_brand == "aubo":
            import platform
            if platform.system() == "Windows" and platform.python_version().rsplit(".", 1)[0] != "3.7":
                log.error_show(f"aubo机器人在Windows下只支持python 3.7.x版本, 当前python版本为{platform.python_version()}")
                raise Exception(f"aubo机器人在Windows下只支持python 3.7.x版本, 当前python版本为{platform.python_version()}")
            from RobotControl.aubo_ctl import AuboControl
            robot_control = AuboControl(default_robot=True,
                                        max_line_vel=0.2, max_line_acc=0.1,
                                        max_angular_vel=40, max_angular_acc=20,
                                        max_joint_vel=90, max_joint_acc=50)
            running_value.value = -1
            log.finish_show(f"机械臂{robot_brand}初始化成功")

        elif robot_brand == "dobot":
            robot_state = DobotApiState()
            robot_control = DobotControl(default_robot=True,
                                         logger=log,
                                         joint_max_vel=70, joint_max_acc=70,
                                         line_max_vel=50, line_max_acc=50,
                                         state_data_array=robot_state.data_array)
            running_value.value = -1
            log.finish_show(f"机械臂{robot_brand}初始化成功")

        elif robot_brand == "moveit2":
            from RobotControl.moveit2_ctl import Moveit2Control
            robot_control = Moveit2Control(logger=log)
            running_value.value = -1
            log.finish_show(f"{robot_brand}初始化成功")

        else:
            robot_control = None
            log.error_show(f"未找到机器人{robot_brand}")

    except Exception as e:
        running_value.value = -2
        log.error_show(f"机械臂初始化失败{str(e)}")
        return

    robot_cmd_thread = threading.Thread(target=robot_cmd,
                                        args=(robot_control, log),
                                        daemon=True)
    robot_cmd_thread.start()

    fs = FastSwitcher(str(Path(__file__).parent / "config/fs_pose.json"),
                      str(Path(__file__).parent / f"config/init_pose_joint.txt"),
                      robot_control)
    while True:
        zmq_sever.recv_msg_dict()

        if zmq_sever.is_exist_in_dict("grasper_select"):
            param = zmq_sever.robot_msg_dict["grasper_select"]
            grasper_select = param["num"]
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
                zmq_sever.send_recv_response({"grasper_select": True})

            except Exception as e:
                log.error_show(f"夹爪初始化失败{str(e)}")
                grasp_init = False
                hand_ctl = None
                zmq_sever.send_recv_response({"grasper_select": False})

            continue

        if zmq_sever.is_exist_in_dict("set_tool_end"):
            param = zmq_sever.robot_msg_dict["set_tool_end"]
            robot_control.tool_end = dict(pos=param["pos"], ori=param["ori"])
            zmq_sever.send_recv_response({"set_tool_end": True})
            continue

        if zmq_sever.is_exist_in_dict("move_to_pose"):
            param = zmq_sever.robot_msg_dict["move_to_pose"]
            result_value.value = 0
            try:
                running_value.value = 1
                pose_rt = robot_control.pos_to_rmatrix(param["pos"], param["ori"])
                robot_control.move_to_waypoints(waypoints=[pose_rt], coor=param["coor"],
                                                move_style=param["move_style"], offset=param["offset"],
                                                is_move_offset=param["is_move_offset"],
                                                check_joints_degree_range=param["check_joints_degree_range"])
                zmq_sever.send_recv_response({"move_to_pose": True})
                result_value.value = 1
                running_value.value = -1

            except Exception as e:
                zmq_sever.send_recv_response({"move_to_pose": False})
                log.error_show(f"机械臂位置移动失败,{e}")
                result_value.value = -1
                running_value.value = -1

            continue

        if zmq_sever.is_exist_in_dict("move_to_joint"):
            param = zmq_sever.robot_msg_dict["move_to_joint"]
            result_value.value = 0
            try:
                running_value.value = 1
                robot_control.move_to_waypoints_in_joint(waypoints_joint=[param["joints"]],
                                                         move_style=param["move_style"],
                                                         check_joints_degree_range=param[
                                                             "check_joints_degree_range"])
                zmq_sever.send_recv_response({"move_to_joint": True})
                result_value.value = 1
                running_value.value = -1
            except Exception as e:
                log.error_show(f"机械臂关节移动失败{str(e)}")
                zmq_sever.send_recv_response({"move_to_joint": False})
                result_value.value = -1
                running_value.value = -1

            continue

        if zmq_sever.is_exist_in_dict("move_offset"):
            param = zmq_sever.robot_msg_dict["move_offset"]
            try:
                running_value.value = 1
                robot_control.move_offset(offset=param["offset"])
                zmq_sever.send_recv_response({"move_offset": True})
                result_value.value = 1
                running_value.value = -1
            except Exception as e:
                log.error_show(f"机械臂偏移移动失败{e}")
                zmq_sever.send_recv_response({"move_offset": False})
                result_value.value = -1
                running_value.value = -1
            continue

        if zmq_sever.is_exist_in_dict("grasper_execute") and grasp_init is True:
            param = zmq_sever.robot_msg_dict["grasper_execute"]
            try:
                if param["is_grasp"] is True:
                    hand_ctl.grasp(pos=param["pos"], force=param["force"])
                else:
                    hand_ctl.release(pos=param["pos"])
                zmq_sever.send_recv_response({"grasper_execute": True})
            except Exception as e:
                log.error_show(f"抓取失败，{e}")
                zmq_sever.send_recv_response({"grasper_execute": False})
            continue

        if zmq_sever.is_exist_in_dict("sucker_execute") and grasp_init is True:
            param = zmq_sever.robot_msg_dict["sucker_execute"]
            try:
                if param["is_suck"] is True:
                    hand_ctl.suck()
                else:
                    hand_ctl.release_suck()
                zmq_sever.send_recv_response({"sucker_execute": True})

            except Exception as e:
                log.error_show(f"抓取失败，{e}")
                zmq_sever.send_recv_response({"sucker_execute": False})
            continue

        if zmq_sever.is_exist_in_dict("switch_grasper"):
            param = zmq_sever.robot_msg_dict["switch_grasper"]
            try:
                if param["release_num"] != 0:
                    if fs.release_switcher(param["release_num"]) and fs.connect_switcher(param["connect_num"]):
                        zmq_sever.send_recv_response({"switch_grasper": True})
                    else:
                        zmq_sever.send_recv_response({"switch_grasper": False})
                else:
                    if fs.connect_switcher(param["connect_num"]):
                        zmq_sever.send_recv_response({"switch_grasper": True})
                    else:
                        zmq_sever.send_recv_response({"switch_grasper": False})
            except Exception as e:
                log.error_show(f"切换夹爪失败，{e}")
                zmq_sever.send_recv_response({"switch_grasper": False})
            continue

        if zmq_sever.is_exist_in_dict("terminate_robot"):
            try:
                zmq_sever.send_recv_response({"terminate_robot": True})
            except Exception as e:
                log.error_show(f"结束机器人失败{str(e)}")
                zmq_sever.send_recv_response({"terminate_robot": False})
            finally:
                break


if __name__ == "__main__":
    zmq_sever_process(sys_argv=sys.argv)
