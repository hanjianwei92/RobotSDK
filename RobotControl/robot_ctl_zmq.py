import json
import zmq
import threading
import sys
import multiprocessing
from pathlib import Path
from typing import Optional
from RobotControl import RobotNode, CasLogger, FastSwitcher


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


def robot_cmd(robot_control: RobotNode, log):
    zmq_sever = RobotSeverZMQ(port=8101)
    while True:
        zmq_sever.recv_msg_dict()
        if zmq_sever.is_exist_in_dict("stop_move"):
            try:
                robot_control.stop_move()
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
                error_id = robot_control.clear_error()
                zmq_sever.send_recv_response({"ClearError": True})
            except Exception as e:
                log.error_show(f"清除dobot错误失败{str(e)}")
                zmq_sever.send_recv_response({"ClearError": False})
            continue

        if zmq_sever.is_exist_in_dict("StartDrag") and robot_control.robot_brand == "dobot":
            param = zmq_sever.robot_msg_dict["StartDrag"]
            try:
                error_id = robot_control.start_drag(status=param["status"])
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

    log = CasLogger('logfiles/robot_move_log.log')
    robot_control = RobotNode(robot_brand=robot_brand, info_queue=log.info_queue,
                              error_queue=log.error_queue, tool_end=dict(pos=(0, 0, 0), ori=(0, 0, 0)))

    zmq_sever = RobotSeverZMQ(port=8100, log=log)

    robot_state_process = multiprocessing.Process(target=robot_state_feedback,
                                                  args=(robot_control.running_value, robot_control.result_value),
                                                  daemon=True)
    robot_state_process.start()

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
                robot_control.switch_grasper(grasper_select)
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
            pose_rt = robot_control.pos_to_rmatrix(param["pos"], param["ori"])
            result = robot_control.move_pose(pose_list=[pose_rt], coor=param["coor"],
                                             move_style=param["move_style"], offset=param["offset"],
                                             check_joints_range=param["check_joints_degree_range"])

            if result is True:
                zmq_sever.send_recv_response({"move_to_pose": True})
            else:
                zmq_sever.send_recv_response({"move_to_pose": False})
                log.error_show(f"机械臂位置移动失败,{e}")

            continue

        if zmq_sever.is_exist_in_dict("move_to_joint"):
            param = zmq_sever.robot_msg_dict["move_to_joint"]
            result = robot_control.move_joint(joint_list=[param["joints"]],
                                              move_style=param["move_style"],
                                              check_joints_range=param["check_joints_degree_range"])
            if result is True:
                zmq_sever.send_recv_response({"move_to_joint": True})

            else:
                log.error_show(f"机械臂关节移动失败{str(e)}")
                zmq_sever.send_recv_response({"move_to_joint": False})
            continue

        if zmq_sever.is_exist_in_dict("move_offset"):
            param = zmq_sever.robot_msg_dict["move_offset"]

            result = robot_control.move_offset(offset=param["offset"])
            if result is True:
                zmq_sever.send_recv_response({"move_offset": True})
            else:
                log.error_show(f"机械臂偏移移动失败{e}")
                zmq_sever.send_recv_response({"move_offset": False})

            continue

        if zmq_sever.is_exist_in_dict("grasper_execute") and grasp_init is True:
            param = zmq_sever.robot_msg_dict["grasper_execute"]
            try:
                if param["is_grasp"] is True:
                    robot_control.grasper_execute(is_grasp=True, pos=param["pos"], force=param["force"])
                else:
                    robot_control.grasper_execute(is_grasp=False, pos=param["pos"])
                zmq_sever.send_recv_response({"grasper_execute": True})
            except Exception as e:
                log.error_show(f"抓取失败，{e}")
                zmq_sever.send_recv_response({"grasper_execute": False})
            continue

        if zmq_sever.is_exist_in_dict("sucker_execute") and grasp_init is True:
            param = zmq_sever.robot_msg_dict["sucker_execute"]
            try:
                if param["is_suck"] is True:
                    robot_control.sucker_execute(is_suck=True)
                else:
                    robot_control.sucker_execute(is_suck=False)
                zmq_sever.send_recv_response({"sucker_execute": True})

            except Exception as e:
                log.error_show(f"抓取失败，{e}")
                zmq_sever.send_recv_response({"sucker_execute": False})
            continue

        if zmq_sever.is_exist_in_dict("switch_grasper"):
            param = zmq_sever.robot_msg_dict["switch_grasper"]
            if fs.connect_switcher(param["connect_num"]):
                zmq_sever.send_recv_response({"switch_grasper": True})
            else:
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
