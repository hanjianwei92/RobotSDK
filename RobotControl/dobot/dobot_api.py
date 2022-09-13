import socket
import numpy as np
import time
import json
import struct
from .alarm_controller import alarm_controller_list
from .alarm_servo import alarm_servo_list
from multiprocessing import Process, Array, Value
from ctypes import c_int32, c_int8

# Port Feedback
MyType = np.dtype([(
    'len',
    np.int64,
), (
    'digital_input_bits',
    np.uint64,
), (
    'digital_output_bits',
    np.uint64,
), (
    'robot_mode',
    np.uint64,
), (
    'time_stamp',
    np.uint64,
), (
    'time_stamp_reserve_bit',
    np.uint64,
), (
    'test_value',
    np.uint64,
), (
    'test_value_keep_bit',
    np.float64,
), (
    'speed_scaling',
    np.float64,
), (
    'linear_momentum_norm',
    np.float64,
), (
    'v_main',
    np.float64,
), (
    'v_robot',
    np.float64,
), (
    'i_robot',
    np.float64,
), (
    'i_robot_keep_bit1',
    np.float64,
), (
    'i_robot_keep_bit2',
    np.float64,
), ('tool_accelerometer_values', np.float64, (3,)),
    ('elbow_position', np.float64, (3,)),
    ('elbow_velocity', np.float64, (3,)),
    ('q_target', np.float64, (6,)),
    ('qd_target', np.float64, (6,)),
    ('qdd_target', np.float64, (6,)),
    ('i_target', np.float64, (6,)),
    ('m_target', np.float64, (6,)),
    ('q_actual', np.float64, (6,)),
    ('qd_actual', np.float64, (6,)),
    ('i_actual', np.float64, (6,)),
    ('actual_TCP_force', np.float64, (6,)),
    ('tool_vector_actual', np.float64, (6,)),
    ('TCP_speed_actual', np.float64, (6,)),
    ('TCP_force', np.float64, (6,)),
    ('Tool_vector_target', np.float64, (6,)),
    ('TCP_speed_target', np.float64, (6,)),
    ('motor_temperatures', np.float64, (6,)),
    ('joint_modes', np.float64, (6,)),
    ('v_actual', np.float64, (6,)),
    # ('dummy', np.float64, (9, 6))])
    ('hand_type', np.byte, (4,)),
    ('user', np.byte,),
    ('tool', np.byte,),
    ('run_queued_cmd', np.byte,),
    ('pause_cmd_flag', np.byte,),
    ('velocity_ratio', np.byte,),
    ('acceleration_ratio', np.byte,),
    ('jerk_ratio', np.byte,),
    ('xyz_velocity_ratio', np.byte,),
    ('r_velocity_ratio', np.byte,),
    ('xyz_acceleration_ratio', np.byte,),
    ('r_acceleration_ratio', np.byte,),
    ('xyz_jerk_ratio', np.byte,),
    ('r_jerk_ratio', np.byte,),
    ('brake_status', np.byte,),
    ('enable_status', np.byte,),
    ('drag_status', np.byte,),
    ('running_status', np.byte,),
    ('error_status', np.byte,),
    ('jog_status', np.byte,),
    ('robot_type', np.byte,),
    ('drag_button_signal', np.byte,),
    ('enable_button_signal', np.byte,),
    ('record_button_signal', np.byte,),
    ('reappear_button_signal', np.byte,),
    ('jaw_button_signal', np.byte,),
    ('six_force_online', np.byte,),
    ('reserve2', np.byte, (82,)),
    ('m_actual', np.float64, (6,)),
    ('load', np.float64,),
    ('center_x', np.float64,),
    ('center_y', np.float64,),
    ('center_z', np.float64,),
    ('user[6]', np.float64, (6,)),
    ('tool[6]', np.float64, (6,)),
    ('trace_index', np.float64,),
    ('six_force_value', np.float64, (6,)),
    ('target_quaternion', np.float64, (4,)),
    ('actual_quaternion', np.float64, (4,)),
    ('reserve3', np.byte, (24,))])


def robot_info_recv():
    try:
        info_socket = socket.socket()
        info_socket.connect(("192.168.5.1", 30004))
    except socket.error:
        print(socket.error)
        raise Exception(
            f"Unable to set socket connection use port 3004 !", socket.error)
    hasRead = 0
    while True:
        data = bytes()
        while hasRead < 1440:
            temp = info_socket.recv(1440 - hasRead)
            if len(temp) > 0:
                hasRead += len(temp)
                data += temp
        hasRead = 0
        a = np.frombuffer(data, dtype=MyType)
        print(a)
        time.sleep(0.005)


class DobotApi:
    def __init__(self, ip, port, *args):
        self.ip = ip
        self.port = port
        self.socket_dobot = 0
        self.text_log = None
        if args:
            self.text_log = args[0]

        self.alarm_controller_dict = self.convert_dict(alarm_controller_list)
        self.alarm_servo_dict = self.convert_dict(alarm_servo_list)

        if self.port == 29999 or self.port == 30003 or self.port == 30004:
            try:
                self.socket_dobot = socket.socket()
                self.socket_dobot.connect((self.ip, self.port))
            except socket.error as e:
                raise Exception(
                    f"Unable to set socket connection use port {self.port} {e}!", socket.error)
        else:
            raise Exception(
                f"Connect to dashboard server need use port {self.port} !")

    def convert_dict(self, alarm_list):
        alarm_dict = {}
        for i in alarm_list:
            alarm_dict[i["id"]] = i
        return alarm_dict

    def log(self, text):
        if self.text_log:
            self.text_log.info_show(text)
        else:
            print(text)

    def send_data(self, string):
        try:
            self.socket_dobot.send(str.encode(string, 'utf-8'))
        except Exception as e:
            self.log(f"Send to 192.168.5.1:{self.port}: {string} Failed {e}, Please reconnect robot")

    def wait_reply(self, wait_error_info: bool = True) -> (int, list):
        """
        Read the return value
        """
        data_str = ""
        try:
            data = self.socket_dobot.recv(1024)
            data_str = str(data, encoding="utf-8")
            result = []
            error_id = int(data_str.split(',')[0])
            if error_id != 0:
                self.log(f'Execute {data_str.split("}")[1].strip(",")} failed, Error id is {error_id}')
            result_data = data_str.split("{")[1].split("}")[0]
            if not result_data:
                result = None
            else:
                for value in result_data.split(','):
                    result.append(float(value))
            if wait_error_info is True:
                error_info = self.GetErrorID()
                if error_info != "":
                    raise Exception(error_info)
            return error_id, result
        except Exception as e:
            self.log(f'Receive from 192.168.5.1:{self.port}: {data_str} Failed{e}, Please reconnect robot')

    def close(self):
        """
        Close the port
        """
        if self.socket_dobot != 0:
            self.socket_dobot.close()

    def GetErrorID(self):
        """
        Get robot error code
        """
        string = "GetErrorID()"
        self.send_data(string)
        data = self.socket_dobot.recv(1024)
        data_str = str(data, encoding="utf-8")
        error_data = data_str.split('{')[1].split('}')[0]
        error_list = json.loads(error_data)
        error_info = ""
        if error_list[0]:
            for i in error_list[0]:
                if i in self.alarm_controller_dict.keys() and i != 0:
                    error_info += f"Type: Controller Error \n" \
                                  f"INFO:{self.alarm_controller_dict[i]['zh_CN']['description']}\n" \
                                  f"Solution:{self.alarm_controller_dict[i]['zh_CN']['solution']}\n"

        for m in range(1, len(error_list)):
            if error_list[m]:
                for n in range(len(error_list[m])):
                    error_info += f"Type: Servo Error \n" \
                                  f"INFO:{self.alarm_servo_dict[n]['zh_CN']['description']}\n" \
                                  f"Solution:{self.alarm_servo_dict[n]['zh_CN']['solution']}\n"

        return error_info

    def __del__(self):
        self.close()


class DobotApiDashboard(DobotApi):
    """
    Define class dobot_api_dashboard to establish a connection to Dobot
    """

    def EnableRobot(self):
        """
        Enable the robot
        """
        string = "EnableRobot()"
        self.send_data(string)
        return self.wait_reply()

    def DisableRobot(self):
        """
        Disabled the robot
        """
        string = "DisableRobot()"
        self.send_data(string)
        return self.wait_reply()

    def ClearError(self):
        """
        Clear controller alarm information
        """
        string = "ClearError()"
        self.send_data(string)
        return self.wait_reply()

    def ResetRobot(self):
        """
        Robot stop
        """
        string = "ResetRobot()"
        self.send_data(string)
        return self.wait_reply()

    def SpeedFactor(self, speed):
        """
        Setting the Global rate
        speed:Rate value(Value range:1~100)
        """
        string = "SpeedFactor({:d})".format(speed)
        self.send_data(string)
        return self.wait_reply()

    def User(self, index):
        """
        Select the calibrated user coordinate system
        index : Calibrated index of user coordinates
        """
        string = "User({:d})".format(index)
        self.send_data(string)
        return self.wait_reply()

    def Tool(self, index):
        """
        Select the calibrated tool coordinate system
        index : Calibrated index of tool coordinates
        """
        string = "Tool({:d})".format(index)
        self.send_data(string)
        return self.wait_reply()

    def RobotMode(self):
        """
        View the robot status
        """
        string = "RobotMode()"
        self.send_data(string)
        return self.wait_reply()

    def PayLoad(self, weight, inertia):
        """
        Setting robot load
        weight : The load weight
        inertia: The load moment of inertia
        """
        string = "PayLoad({:f},{:f})".format(weight, inertia)
        self.send_data(string)
        return self.wait_reply()

    def DO(self, index, status):
        """
        Set digital signal output (Queue instruction)
        index : Digital output index (Value range:1~24)
        status : Status of digital signal output port(0:Low level，1:High level
        """
        string = "DO({:d},{:d})".format(index, status)
        self.send_data(string)
        return self.wait_reply()

    def DOExecute(self, index, status):
        """
        Set digital signal output (Instructions immediately)
        index : Digital output index (Value range:1~24)
        status : Status of digital signal output port(0:Low level，1:High level)
        """
        string = "DOExecute({:d},{:d})".format(index, status)
        self.send_data(string)
        return self.wait_reply()

    def ToolDO(self, index, status):
        """
        Set terminal signal output (Queue instruction)
        index : Terminal output index (Value range:1~2)
        status : Status of digital signal output port(0:Low level，1:High level)
        """
        string = "ToolDO({:d},{:d})".format(index, status)
        self.send_data(string)
        return self.wait_reply()

    def ToolDOExecute(self, index, status):
        """
        Set terminal signal output (Instructions immediately)
        index : Terminal output index (Value range:1~2)
        status : Status of digital signal output port(0:Low level，1:High level)
        """
        string = "ToolDOExecute({:d},{:d})".format(index, status)
        self.send_data(string)
        return self.wait_reply()

    def AO(self, index, val):
        """
        Set analog signal output (Queue instruction)
        index : Analog output index (Value range:1~2)
        val : Voltage value (0~10)
        """
        string = "AO({:d},{:f})".format(index, val)
        self.send_data(string)
        return self.wait_reply()

    def AOExecute(self, index, val):
        """
        Set analog signal output (Instructions immediately)
        index : Analog output index (Value range:1~2)
        val : Voltage value (0~10)
        """
        string = "AOExecute({:d},{:f})".format(index, val)
        self.send_data(string)
        return self.wait_reply()

    def AccJ(self, speed):
        """
        Set joint acceleration ratio (Only for MovJ, MovJIO, MovJR, JointMovJ commands)
        speed : Joint acceleration ratio (Value range:1~100)
        """
        string = "AccJ({:d})".format(speed)
        self.send_data(string)
        return self.wait_reply()

    def AccL(self, speed):
        """
        Set the coordinate system acceleration ratio (Only for MovL, MovLIO, MovLR, Jump, Arc, Circle commands)
        speed : Cartesian acceleration ratio (Value range:1~100)
        """
        string = "AccL({:d})".format(speed)
        self.send_data(string)
        return self.wait_reply()

    def SpeedJ(self, speed):
        """
        Set joint speed ratio (Only for MovJ, MovJIO, MovJR, JointMovJ commands)
        speed : Joint velocity ratio (Value range:1~100)
        """
        string = "SpeedJ({:d})".format(speed)
        self.send_data(string)
        return self.wait_reply()

    def SpeedL(self, speed):
        """
        Set the cartesian acceleration ratio (Only for MovL, MovLIO, MovLR, Jump, Arc, Circle commands)
        speed : Cartesian acceleration ratio (Value range:1~100)
        """
        string = "SpeedL({:d})".format(speed)
        self.send_data(string)
        return self.wait_reply()

    def Arch(self, index):
        """
        Set the Jump gate parameter index (This index contains: start point lift height, maximum lift height, end point drop height)
        index : Parameter index (Value range:0~9)
        """
        string = "Arch({:d})".format(index)
        self.send_data(string)
        return self.wait_reply()

    def CP(self, ratio):
        """
        Set smooth transition ratio
        ratio : Smooth transition ratio (Value range:1~100)
        """
        string = "CP({:d})".format(ratio)
        self.send_data(string)
        return self.wait_reply()

    def LimZ(self, value):
        """
        Set the maximum lifting height of door type parameters
        value : Maximum lifting height (Highly restricted:Do not exceed the limit position of the z-axis of the manipulator)
        """
        string = "LimZ({:d})".format(value)
        self.send_data(string)
        return self.wait_reply()

    def SetArmOrientation(self, r, d, n, cfg):
        """
        Set the hand command
        r : Mechanical arm direction, forward/backward (1:forward -1:backward)
        d : Mechanical arm direction, up elbow/down elbow (1:up elbow -1:down elbow)
        n : Whether the wrist of the mechanical arm is flipped (1:The wrist does not flip -1:The wrist flip)
        cfg :Sixth axis Angle identification
            (1, - 2... : Axis 6 Angle is [0,-90] is -1; [90, 180] - 2; And so on
            1, 2... : axis 6 Angle is [0,90] is 1; [90180] 2; And so on)
        """
        string = "SetArmOrientation({:d},{:d},{:d},{:d})".format(r, d, n, cfg)
        self.send_data(string)
        return self.wait_reply()

    def PowerOn(self):
        """
        Powering on the robot
        Note: It takes about 10 seconds for the robot to be enabled after it is powered on.
        """
        string = "PowerOn()"
        self.send_data(string)
        return self.wait_reply()

    def RunScript(self, project_name):
        """
        Run the script file
        project_name ：Script file name
        """
        string = "RunScript({:s})".format(project_name)
        self.send_data(string)
        return self.wait_reply()

    def StopScript(self):
        """
        Stop scripts
        """
        string = "StopScript()"
        self.send_data(string)
        return self.wait_reply()

    def PauseScript(self):
        """
        Pause the script
        """
        string = "PauseScript()"
        self.send_data(string)
        return self.wait_reply()

    def ContinueScript(self):
        """
        Continue running the script
        """
        string = "ContinueScript()"
        self.send_data(string)
        return self.wait_reply()

    def GetHoldRegs(self, id, addr, count, type):
        """
        Read hold register
        id :Secondary device NUMBER (A maximum of five devices can be supported. The value ranges from 0 to 4
            Set to 0 when accessing the internal slave of the controller)
        addr :Hold the starting address of the register (Value range:3095~4095)
        count :Reads the specified number of types of data (Value range:1~16)
        type :The data type
            If null, the 16-bit unsigned integer (2 bytes, occupying 1 register) is read by default
            "U16" : reads 16-bit unsigned integers (2 bytes, occupying 1 register)
            "U32" : reads 32-bit unsigned integers (4 bytes, occupying 2 registers)
            "F32" : reads 32-bit single-precision floating-point number (4 bytes, occupying 2 registers)
            "F64" : reads 64-bit double precision floating point number (8 bytes, occupying 4 registers)
        """
        string = "GetHoldRegs({:d},{:d},{:d},{:s})".format(
            id, addr, count, type)
        self.send_data(string)
        return self.wait_reply()

    def SetHoldRegs(self, id, addr, count, table, type):
        """
        Write hold register
        id :Secondary device NUMBER (A maximum of five devices can be supported. The value ranges from 0 to 4
            Set to 0 when accessing the internal slave of the controller)
        addr :Hold the starting address of the register (Value range:3095~4095)
        count :Writes the specified number of types of data (Value range:1~16)
        type :The data type
            If null, the 16-bit unsigned integer (2 bytes, occupying 1 register) is read by default
            "U16" : reads 16-bit unsigned integers (2 bytes, occupying 1 register)
            "U32" : reads 32-bit unsigned integers (4 bytes, occupying 2 registers)
            "F32" : reads 32-bit single-precision floating-point number (4 bytes, occupying 2 registers)
            "F64" : reads 64-bit double precision floating point number (8 bytes, occupying 4 registers)
        """
        value = "{"
        for v in table:
            value += f"{v:d},"
        value = value.strip(",")
        value += "}"
        string = f"SetHoldRegs({id:d},{addr:d},{count:d},{value:s},{type:s})"
        self.send_data(string)
        return self.wait_reply()

    def SetCoils(self, id, addr, count, table):
        value = "{"
        for v in table:
            value += f"{v:d},"
            value = value.strip(",")
        value += "}"
        string = f"SetCoils({id:d},{addr:d},{count:d},{value:s})"
        self.send_data(string)
        return self.wait_reply()

    def GetCoils(self, id, addr, count):
        string = f"SetCoils({id:d},{addr:d},{count:d})"
        self.send_data(string)
        return self.wait_reply()

    def ModbusCreate(self, ip, port, slave_id, isRTU):
        string = f"ModbusCreate({ip:s},{port:d},{slave_id:d},{isRTU:d})"
        self.send_data(string)
        return self.wait_reply()

    def ModbusClose(self, index):
        string = f"ModbusClose({index:d})"
        self.send_data(string)
        return self.wait_reply()

    def SetTerminal485(self, baudRate=115200, dataLen=8, parityBit='N', stopBit=1):
        string = f"SetTerminal485({baudRate:d},{dataLen:d},{parityBit:s},{stopBit:d})"
        self.send_data(string)
        return self.wait_reply()

    def LoadSwitch(self, statue):
        string = f"LoadSwitch({statue:d})"
        self.send_data(string)
        return self.wait_reply()

    def SetSafeSkin(self, status):
        string = f"SetSafeSkin({status:d})"
        self.send_data(string)
        return self.wait_reply()

    def PositiveSolution(self, joints_list: list):
        string = f"PositiveSolution({joints_list[0]:f},{joints_list[1]:f}," \
                 f"{joints_list[2]:f},{joints_list[3]:f},{joints_list[4]:f},{joints_list[5]:f}," \
                 f"0,0)"
        self.send_data(string)
        error_id, pos = self.wait_reply()
        pos[0] /= 1000.0
        pos[1] /= 1000.0
        pos[2] /= 1000.0
        return error_id, pos

    def InverseSolution(self, pos):
        """
        :param pos: [x, y, z, Rx, Ry, Rz] unit: mm, °
        :return:
        """
        pos[0] *= 1000.0
        pos[1] *= 1000.0
        pos[2] *= 1000.0
        string = f"InverseSolution({pos[0]:f},{pos[1]:f},{pos[2]:f},{pos[3]:f},{pos[4]:f},{pos[5]:f},{pos[6]:d}," \
                 f"{pos[7]:d},{pos[8]:d},{{{pos[9]},{pos[10]},{pos[11]},{pos[12]},{pos[13]},{pos[14]}}})"
        self.send_data(string)
        return self.wait_reply()

    def SetCollisionLevel(self, level):
        """
        :param level: 0-5,0 represent collision detection closed
        :return:
        """
        string = f"SetCollisionLevel({level:d})"
        self.send_data(string)
        return self.wait_reply()

    def GetAngle(self):
        """
        :return:六个关节角度 °
        """
        string = f"GetAngle()"
        self.send_data(string)
        return self.wait_reply()

    def GetPose(self):
        """
        :return:位姿（单位：mm、°）
        """
        string = f"GetPose()"
        self.send_data(string)
        error_id, pos = self.wait_reply()
        pos[0] /= 1000.0
        pos[1] /= 1000.0
        pos[2] /= 1000.0
        return error_id, pos

    def EmergencyStop(self):
        string = f"EmergencyStop()"
        self.send_data(string)
        return self.wait_reply()

    def StartDrag(self, status):
        string1 = "StartDrag()"
        string0 = "StopDrag()"
        if status == 1:
            self.send_data(string1)
        else:
            self.send_data(string0)
        return self.wait_reply()


class DobotApiMove(DobotApi):
    """
    Define class dobot_api_move to establish a connection to Dobot
    """

    def MovJ(self, pos: list):
        """
        Joint motion interface (point-to-point motion mode)
        x: A number in the Cartesian coordinate system x
        y: A number in the Cartesian coordinate system y
        z: A number in the Cartesian coordinate system z
        rx: Position of Rx axis in Cartesian coordinate system
        ry: Position of Ry axis in Cartesian coordinate system
        rz: Position of Rz axis in Cartesian coordinate system
        """
        pos[0] *= 1000.0
        pos[1] *= 1000.0
        pos[2] *= 1000.0
        string = "MovJ({:f},{:f},{:f},{:f},{:f},{:f})".format(
            pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
        self.send_data(string)
        return self.wait_reply(False)

    def MovL(self, pos: list):
        """
        Coordinate system motion interface (linear motion mode)
        x: A number in the Cartesian coordinate system x
        y: A number in the Cartesian coordinate system y
        z: A number in the Cartesian coordinate system z
        rx: Position of Rx axis in Cartesian coordinate system
        ry: Position of Ry axis in Cartesian coordinate system
        rz: Position of Rz axis in Cartesian coordinate system
        """
        pos[0] *= 1000.0
        pos[1] *= 1000.0
        pos[2] *= 1000.0
        string = "MovL({:f},{:f},{:f},{:f},{:f},{:f})".format(
            pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
        self.send_data(string)
        return self.wait_reply(False)

    def JointMovJ(self, joint: list):
        """
        Joint motion interface (linear motion mode)
        j1~j6:Point position values on each joint
        """
        string = "JointMovJ({:f},{:f},{:f},{:f},{:f},{:f})".format(
            joint[0], joint[1], joint[2], joint[3], joint[4], joint[5])
        self.send_data(string)
        return self.wait_reply(False)

    def Jump(self):
        print("待定")

    def RelMovJ(self, offset1, offset2, offset3, offset4, offset5, offset6):
        """
        Offset motion interface (point-to-point motion mode)
        j1~j6:Point position values on each joint
        """
        string = "RelMovJ({:f},{:f},{:f},{:f},{:f},{:f})".format(
            offset1, offset2, offset3, offset4, offset5, offset6)
        self.send_data(string)
        return self.wait_reply(False)

    def RelMovL(self, offsetX, offsetY, offsetZ):
        """
        Offset motion interface (point-to-point motion mode)
        x: Offset in the Cartesian coordinate system x
        y: offset in the Cartesian coordinate system y
        z: Offset in the Cartesian coordinate system Z
        """
        string = "RelMovL({:f},{:f},{:f})".format(offsetX, offsetY, offsetZ)
        self.send_data(string)
        return self.wait_reply(False)

    def MovLIO(self, x, y, z, a, b, c, *dynParams):
        """
        Set the digital output port state in parallel while moving in a straight line
        x: A number in the Cartesian coordinate system x
        y: A number in the Cartesian coordinate system y
        z: A number in the Cartesian coordinate system z
        a: A number in the Cartesian coordinate system a
        b: A number in the Cartesian coordinate system b
        c: a number in the Cartesian coordinate system c
        *dynParams :Parameter Settings（Mode、Distance、Index、Status）
                    Mode :Set Distance mode (0: Distance percentage; 1: distance from starting point or target point)
                    Distance :Runs the specified distance（If Mode is 0, the value ranges from 0 to 100；When Mode is 1, if the value is positive,
                             it indicates the distance from the starting point. If the value of Distance is negative, it represents the Distance from the target point）
                    Index ：Digital output index （Value range：1~24）
                    Status ：Digital output state（Value range：0/1）
        """
        # example： MovLIO(0,50,0,0,0,0,(0,50,1,0),(1,1,2,1))
        string = "MovLIO({:f},{:f},{:f},{:f},{:f},{:f}".format(
            x, y, z, a, b, c)
        print(type(dynParams), dynParams)
        for params in dynParams:
            print(type(params), params)
            string = string + ",{{{:d},{:d},{:d},{:d}}}".format(
                params[0], params[1], params[2], params[3])
        string = string + ")"
        self.send_data(string)
        return self.wait_reply()

    def MovJIO(self, x, y, z, a, b, c, *dynParams):
        """
        Set the digital output port state in parallel during point-to-point motion
        x: A number in the Cartesian coordinate system x
        y: A number in the Cartesian coordinate system y
        z: A number in the Cartesian coordinate system z
        a: A number in the Cartesian coordinate system a
        b: A number in the Cartesian coordinate system b
        c: a number in the Cartesian coordinate system c
        *dynParams :Parameter Settings（Mode、Distance、Index、Status）
                    Mode :Set Distance mode (0: Distance percentage; 1: distance from starting point or target point)
                    Distance :Runs the specified distance（If Mode is 0, the value ranges from 0 to 100；When Mode is 1, if the value is positive,
                             it indicates the distance from the starting point. If the value of Distance is negative, it represents the Distance from the target point）
                    Index ：Digital output index （Value range：1~24）
                    Status ：Digital output state（Value range：0/1）
        """
        # example： MovJIO(0,50,0,0,0,0,(0,50,1,0),(1,1,2,1))
        string = "MovJIO({:f},{:f},{:f},{:f},{:f},{:f}".format(
            x, y, z, a, b, c)
        for params in dynParams:
            print(type(params), params)
            string = string + ",{{{:d},{:d},{:d},{:d}}}".format(
                params[0], params[1], params[2], params[3])
        string = string + ")"
        self.send_data(string)
        return self.wait_reply()

    def Arc(self, x1, y1, z1, a1, b1, c1, x2, y2, z2, a2, b2, c2):
        """
        Circular motion instruction
        x1, y1, z1, a1, b1, c1 :Is the point value of intermediate point coordinates
        x2, y2, z2, a2, b2, c2 :Is the value of the end point coordinates
        Note: This instruction should be used together with other movement instructions
        """
        string = "Arc({:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f})".format(
            x1, y1, z1, a1, b1, c1, x2, y2, z2, a2, b2, c2)
        self.send_data(string)
        return self.wait_reply()

    def Circle(self, count, x1, y1, z1, a1, b1, c1, x2, y2, z2, a2, b2, c2):
        """
        Full circle motion command
        count：Run laps
        x1, y1, z1, a1, b1, c1 :Is the point value of intermediate point coordinates
        x2, y2, z2, a2, b2, c2 :Is the value of the end point coordinates
        Note: This instruction should be used together with other movement instructions
        """
        string = "Circle({:d},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f})".format(
            count, x1, y1, z1, a1, b1, c1, x2, y2, z2, a2, b2, c2)
        self.send_data(string)
        return self.wait_reply()

    def ServoJ(self, j1, j2, j3, j4, j5, j6):
        """
        Dynamic follow command based on joint space
        j1~j6:Point position values on each joint
        """
        string = "ServoJ({:f},{:f},{:f},{:f},{:f},{:f})".format(
            j1, j2, j3, j4, j5, j6)
        self.send_data(string)
        return self.wait_reply()

    def ServoP(self, x, y, z, a, b, c):
        """
        Dynamic following command based on Cartesian space
        x, y, z, a, b, c :Cartesian coordinate point value
        """
        string = "ServoP({:f},{:f},{:f},{:f},{:f},{:f})".format(
            x, y, z, a, b, c)
        self.send_data(string)
        # return self.wait_reply()

    def MoveJog(self, axis_id, *dynParams):
        """
        Joint motion
        axis_id: Joint motion axis, optional string value:
            J1+ J2+ J3+ J4+ J5+ J6+
            J1- J2- J3- J4- J5- J6-
            X+ Y+ Z+ Rx+ Ry+ Rz+
            X- Y- Z- Rx- Ry- Rz-
        *dynParams: Parameter Settings（coord_type, user_index, tool_index）
                    coord_type: 1: User coordinate 2: tool coordinate (default value is 1)
                    user_index: user index is 0 ~ 9 (default value is 0)
                    tool_index: tool index is 0 ~ 9 (default value is 0)
        """
        string = f"MoveJog({axis_id}"
        for params in dynParams:
            print(type(params), params)
            string = string + ", CoordType={:d}, User={:d}, Tool={:d}".format(
                params[0], params[1], params[2])
        string = string + ")"
        self.send_data(string)
        return self.wait_reply()

    def StartTrace(self, trace_name):
        """
        Trajectory fitting (track file Cartesian points)
        trace_name: track file name (including suffix)
        (The track path is stored in /dobot/userdata/project/process/trajectory/)

        It needs to be used together with `GetTraceStartPose(recv_string.json)` interface
        """
        string = f"StartTrace({trace_name})"
        self.send_data(string)
        return self.wait_reply()

    def StartPath(self, trace_name, const, cart):
        """
        Track reproduction. (track file joint points)
        trace_name: track file name (including suffix)
        (The track path is stored in /dobot/userdata/project/process/trajectory/)
        const: When const = 1, it repeats at a constant speed, and the pause and dead zone in the track will be removed;
               When const = 0, reproduce according to the original speed;
        cart: When cart = 1, reproduce according to Cartesian path;
              When cart = 0, reproduce according to the joint path;

        It needs to be used together with `GetTraceStartPose(recv_string.json)` interface
        """
        string = f"StartPath({trace_name}, {const}, {cart})"
        self.send_data(string)
        return self.wait_reply()

    def StartFCTrace(self, trace_name):
        """
        Trajectory fitting with force control. (track file Cartesian points)
        trace_name: track file name (including suffix)
        (The track path is stored in /dobot/userdata/project/process/trajectory/)

        It needs to be used together with `GetTraceStartPose(recv_string.json)` interface
        """
        string = f"StartFCTrace({trace_name})"
        self.send_data(string)
        return self.wait_reply()

    def Sync(self):
        """
        The blocking program executes the queue instruction and returns after all the queue instructions are executed
        """
        string = "Sync()"
        self.send_data(string)
        return self.wait_reply()

    def RelMovJTool(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, tool, *dynParams):
        """
        The relative motion command is carried out along the tool coordinate system, and the end motion mode is joint motion
        offset_x: X-axis direction offset
        offset_y: Y-axis direction offset
        offset_z: Z-axis direction offset
        offset_rx: Rx axis position
        offset_ry: Ry axis position
        offset_rz: Rz axis position
        tool: Select the calibrated tool coordinate system, value range: 0 ~ 9
        *dynParams: parameter Settings（speed_j, acc_j, user）
                    speed_j: Set joint speed scale, value range: 1 ~ 100
                    acc_j: Set acceleration scale value, value range: 1 ~ 100
                    user: Set user coordinate system index
        """
        string = "RelMovJTool({:f},{:f},{:f},{:f},{:f},{:f}, {:d}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, tool)
        for params in dynParams:
            print(type(params), params)
            string = string + ", SpeedJ={:d}, AccJ={:d}, User={:d}".format(
                params[0], params[1], params[2])
        string = string + ")"
        self.send_data(string)
        return self.wait_reply()

    def RelMovLTool(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, tool, *dynParams):
        """
        Carry out relative motion command along the tool coordinate system, and the end motion mode is linear motion
        offset_x: X-axis direction offset
        offset_y: Y-axis direction offset
        offset_z: Z-axis direction offset
        offset_rx: Rx axis position
        offset_ry: Ry axis position
        offset_rz: Rz axis position
        tool: Select the calibrated tool coordinate system, value range: 0 ~ 9
        *dynParams: parameter Settings（speed_l, acc_l, user）
                    speed_l: Set Cartesian speed scale, value range: 1 ~ 100
                    acc_l: Set acceleration scale value, value range: 1 ~ 100
                    user: Set user coordinate system index
        """
        string = "RelMovLTool({:f},{:f},{:f},{:f},{:f},{:f}, {:d}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, tool)
        for params in dynParams:
            print(type(params), params)
            string = string + ", SpeedJ={:d}, AccJ={:d}, User={:d}".format(
                params[0], params[1], params[2])
        string = string + ")"
        self.send_data(string)
        return self.wait_reply()

    def RelMovJUser(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, user, *dynParams):
        """
        The relative motion command is carried out along the user coordinate system, and the end motion mode is joint motion
        offset_x: X-axis direction offset
        offset_y: Y-axis direction offset
        offset_z: Z-axis direction offset
        offset_rx: Rx axis position
        offset_ry: Ry axis position
        offset_rz: Rz axis position
        user: Select the calibrated user coordinate system, value range: 0 ~ 9
        *dynParams: parameter Settings（speed_j, acc_j, tool）
                    speed_j: Set joint speed scale, value range: 1 ~ 100
                    acc_j: Set acceleration scale value, value range: 1 ~ 100
                    tool: Set tool coordinate system index
        """
        string = "RelMovJUser({:f},{:f},{:f},{:f},{:f},{:f}, {:d}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, user)
        for params in dynParams:
            print(type(params), params)
            string = string + ", SpeedJ={:d}, AccJ={:d}, Tool={:d}".format(
                params[0], params[1], params[2])
        string = string + ")"
        self.send_data(string)
        return self.wait_reply()

    def RelMovLUser(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, user, *dynParams):
        """
        The relative motion command is carried out along the user coordinate system, and the end motion mode is linear motion
        offset_x: X-axis direction offset
        offset_y: Y-axis direction offset
        offset_z: Z-axis direction offset
        offset_rx: Rx axis position
        offset_ry: Ry axis position
        offset_rz: Rz axis position
        user: Select the calibrated user coordinate system, value range: 0 ~ 9
        *dynParams: parameter Settings（speed_l, acc_l, tool）
                    speed_l: Set Cartesian speed scale, value range: 1 ~ 100
                    acc_l: Set acceleration scale value, value range: 1 ~ 100
                    tool: Set tool coordinate system index
        """
        string = "RelMovLUser({:f},{:f},{:f},{:f},{:f},{:f}, {:d}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, user)
        for params in dynParams:
            print(type(params), params)
            string = string + ", SpeedJ={:d}, AccJ={:d}, Tool={:d}".format(
                params[0], params[1], params[2])
        string = string + ")"
        self.send_data(string)
        return self.wait_reply()

    def RelJointMovJ(self, offset1, offset2, offset3, offset4, offset5, offset6, *dynParams):
        """
        The relative motion command is carried out along the joint coordinate system of each axis, and the end motion mode is joint motion
        Offset motion interface (point-to-point motion mode)
        j1~j6:Point position values on each joint
        *dynParams: parameter Settings（speed_j, acc_j, user）
                    speed_j: Set Cartesian speed scale, value range: 1 ~ 100
                    acc_j: Set acceleration scale value, value range: 1 ~ 100
        """
        string = "RelJointMovJ({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset1, offset2, offset3, offset4, offset5, offset6)
        for params in dynParams:
            print(type(params), params)
            string = string + ", SpeedJ={:d}, AccJ={:d}".format(
                params[0], params[1])
        string = string + ")"
        self.send_data(string)
        return self.wait_reply()


class DobotApiState(Process):
    def __init__(self):
        super().__init__()
        self.data_array = Array(c_int32, 360)
        self.error_state = Value(c_int8, 0)
        self.daemon = True
        self.start()

    def run(self) -> None:
        info_socket = socket.socket()
        try:
            info_socket.connect(("192.168.5.1", 30004))
        except socket.error:
            print(f"Unable to set socket connection use port 3004 is {socket.error} !")
            return

        hasRead = 0
        while True:
            data = bytes()
            while hasRead < 1440:
                temp = info_socket.recv(1440 - hasRead)
                if len(temp) > 0:
                    hasRead += len(temp)
                    data += temp
            hasRead = 0
            self.data_array[0:360] = struct.unpack('360i', data)
            robot_state = np.frombuffer(data, dtype=MyType)
            if robot_state['error_status'][0] == 1:
                self.error_state.value = 1

    def get_robot_state(self):
        with self.data_array.get_lock():
            robot_state = struct.pack('360i', *self.data_array)
            return np.frombuffer(robot_state, dtype=MyType)


def get_robot_state_local():
    try:
        info_socket = socket.socket()
        info_socket.connect(("192.168.5.1", 30004))
    except socket.error:
        print(f"Unable to set socket connection use port 30004 is {socket.error} !")
        return
    hasRead = 0
    data = bytes()
    while hasRead < 1440:
        temp = info_socket.recv(1440 - hasRead)
        if len(temp) > 0:
            hasRead += len(temp)
            data += temp
    info_socket.close()
    return np.frombuffer(data, dtype=MyType)


# if __name__ == "__main__":
#     a = DobotApiState()
#     while True:
#         time.sleep(0.5)
#         c = time.time()
#         robot = a.get_robot_state()
#         # robot = get_robot_state_local()
#         # print(f"get time is {time.time() - c}")
#         # print(robot['tool_vector_actual'])
#         print(robot['q_actual'][0][5])
#         print(robot['robot_mode'][0], robot['error_status'][0])
#         print(f"errorstate is {a.error_state.value}")
