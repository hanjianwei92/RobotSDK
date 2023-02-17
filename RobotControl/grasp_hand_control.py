import time
from RobotControl import DobotControl
import modbus_tk.modbus_tcp as modbus_tcp
from modbus_tk import defines
import modbus_tk.modbus_rtu as modbus_rtu
import serial


class Modbus:
    RTU = 0
    TCP_to_RTU = 1
    Dobot = 2


class ModbusCtl:
    def __init__(self, log=None, ip="192.168.5.12", port=9502):
        self.master = modbus_tcp.TcpMaster(host=ip, port=port)
        self.master.set_timeout(3.0)
        self.master.open()
        self.log = log
        # data = [0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0x01, 0x05, 0x00, address, data]
        # a = struct.pack("!" + "B" * 12, *data)
        # self.tcp_client_socket.send(a)

    def write_single_coil(self, address, data):
        try:
            self.master.execute(slave=0x01,
                                function_code=defines.WRITE_SINGLE_COIL,
                                starting_address=address,
                                output_value=data)
        except Exception as e:
            if self.log is None:
                print(str(e))
            else:
                self.log.error_show(str(e))

    def read_input_registers(self, address, read_nums=1):
        try:
            recv_data = self.master.execute(slave=0x01,
                                            function_code=defines.READ_INPUT_REGISTERS,
                                            starting_address=address,
                                            quantity_of_x=read_nums)
            return recv_data
        except Exception as e:
            if self.log is None:
                print(str(e))
            else:
                self.log.error_show(str(e))

    def write_holding_single_register(self, address, data):
        try:
            self.master.execute(slave=0x01,
                                function_code=defines.WRITE_SINGLE_REGISTER,
                                starting_address=address,
                                output_value=data)
        except Exception as e:
            if self.log is None:
                print(str(e))
            else:
                self.log.error_show(str(e))


class UniversalGraspHandCtl(ModbusCtl):
    def __init__(self, log=None, ip="192.168.5.12", port=9502):
        super().__init__(log=log, ip=ip, port=port)

    def grasp(self, pos=0, force=50):
        self.write_single_coil(0x02, 0xFF00)

    def release(self, pos=1000):
        self.write_single_coil(0x02, 0x0000)

    def suck(self):
        self.write_single_coil(0x01, 0xFF00)

    def release_suck(self):
        self.write_single_coil(0x01, 0x0000)


class PressureCtl(ModbusCtl):
    def __init__(self, log=None, ip="192.168.5.12", port=9502):
        super().__init__(log=log, ip=ip, port=port)

    def get_curr_pressure(self):
        recv_data = self.read_input_registers(address=0x05)
        return recv_data[0]

    def set_curr_pressure(self, pressure_value):
        self.write_holding_single_register(address=0x06, data=pressure_value)


class DhModbus:
    def __init__(self,
                 connect_type=Modbus.TCP_to_RTU,
                 ip="192.168.5.12", port=502,
                 rtu_port_name='COM3', baud_rate=115200,
                 dobot_robot: DobotControl = None,
                 log=None):
        self.gripper_ID = 0x02
        self.log = log
        self.connect_type = connect_type

        if connect_type == Modbus.RTU:
            self.master = modbus_rtu.RtuMaster(serial.Serial(port=rtu_port_name, baudrate=baud_rate))
            self.master.set_timeout(5.0)
            # self.master.set_verbose(True)
            self.master.open()

        elif connect_type == Modbus.TCP_to_RTU:
            self.master = modbus_tcp.TcpMaster(host=ip, port=port)
            self.master.set_timeout(5.0)
            # self.master.set_verbose(True)
            self.master.open()

        elif connect_type == Modbus.Dobot:
            if dobot_robot is None:
                dobot_robot = DobotControl()
            self.robot = dobot_robot
            self.robot.robot_ctl.SetTerminal485(baudRate=115200)
            self.master_index = self.robot.robot_ctl.ModbusCreate("127.0.0.1", 60000, self.gripper_ID, True)[0]
        else:
            self.log.error_show("Modbus Connect Type Error")

        self.init_dh_hand()

    def send_cmd(self, address, val):
        try:
            if self.connect_type == Modbus.Dobot:
                self.robot.robot_ctl.SetHoldRegs(self.master_index, address, 1, [val], "U16")

            else:
                self.master.execute(slave=self.gripper_ID,
                                    function_code=defines.WRITE_SINGLE_REGISTER,
                                    starting_address=address,
                                    output_value=val)
        except Exception as e:
            if self.log is None:
                print(str(e))
            else:
                self.log.error_show(str(e))

    def receive_cmd(self, address, read_nums=1):
        try:
            if self.connect_type == Modbus.Dobot:
                data = self.robot.robot_ctl.GetHoldRegs(self.master_index, address, read_nums, "U16")
                if data is None:
                    return None
                recv_data = data[1][0]
            else:
                recv_data = self.master.execute(slave=self.gripper_ID,
                                                function_code=defines.READ_HOLDING_REGISTERS,
                                                starting_address=address,
                                                quantity_of_x=read_nums)
            return recv_data
        except Exception as e:
            if self.log is None:
                print(str(e))
            else:
                self.log.error_show(str(e))

    def init_dh_hand(self):
        self.send_cmd(0x0100, 0xA5)
        time.sleep(3.0)
        self.send_cmd(0x0300, 0x00)
        time.sleep(1.5)

    def set_force(self, value):
        """
        设定抓力值百分比
        Args:
            value:百分比：20-100
        Returns:
        """
        self.send_cmd(0x0101, int(value))

    def set_position(self, value):
        """
        设置夹爪手指位置
        Args:
            value: 米 0-0.145
        Returns:
        """
        value = value / 0.145 * 1000
        self.send_cmd(0x0103, int(value))

    def get_curr_position(self, is_curr: bool = True):
        """
        Returns: 读取当前实时位置或设定位置 千分比
        """
        if is_curr:
            return self.receive_cmd(0x0202)
        else:
            return self.receive_cmd(0x0103)

    def get_curr_grasp_statue(self):
        """
        返回夹爪夹持状态
        Returns:· 00 ：夹爪处于正在运动状态。
                · 01 ：夹爪停止运动，且夹爪未检测到夹到物体。
                · 02 ：夹爪停止运动，且夹爪检测到夹到物体。
                · 03 ：夹爪检测到夹住物体后，发现物体掉落。
        """
        return self.receive_cmd(0x0201)

    def grasp(self, pos=0.0, force=40):
        if force is None:
            force = 40
        self.set_force(force)
        time.sleep(0.5)
        self.set_position(pos)

    def release(self, pos=0.145):
        if pos is None:
            pos = 0.145
        self.set_position(pos)


class RMHandModbus:
    def __init__(self,
                 connect_type=Modbus.TCP_to_RTU,
                 ip="192.168.5.12", port=502,
                 rtu_port_name='COM4', baud_rate=115200,
                 dobot_robot: DobotControl = None,
                 log=None):
        self.gripper_ID = 0x04
        self.log = log
        self.connect_type = connect_type

        if connect_type == Modbus.RTU:
            self.master = modbus_rtu.RtuMaster(serial.Serial(port=rtu_port_name, baudrate=baud_rate))
            self.master.set_timeout(5.0)
            self.master.open()

        elif connect_type == Modbus.TCP_to_RTU:
            self.master = modbus_tcp.TcpMaster(host=ip, port=port)
            self.master.set_timeout(5.0)
            self.master.open()

        elif connect_type == Modbus.Dobot:
            if dobot_robot is None:
                dobot_robot = DobotControl()
            self.robot = dobot_robot
            self.robot.robot_ctl.SetTerminal485(baudRate=115200)
            self.master_index = self.robot.robot_ctl.ModbusCreate("127.0.0.1", 60000, self.gripper_ID, True)[0]

        else:
            self.log.error_show("Modbus Connect Type Error")

        self.init_rm()
        self.release(0.04)

    def send_ctl_cmd(self, address):
        try:
            if self.connect_type == Modbus.Dobot:
                self.robot.robot_ctl.SetCoils(self.master_index, address, 1, [0])
                time.sleep(0.1)
                self.robot.robot_ctl.SetCoils(self.master_index, address, 1, [1])
                time.sleep(0.1)
            else:
                self.master.execute(slave=self.gripper_ID,
                                    function_code=defines.WRITE_SINGLE_COIL,
                                    starting_address=address,
                                    output_value=0)
                time.sleep(0.1)
                self.master.execute(slave=self.gripper_ID,
                                    function_code=defines.WRITE_SINGLE_COIL,
                                    starting_address=address,
                                    output_value=1)
                time.sleep(0.1)
        except Exception as e:
            if self.log is None:
                print(str(e))
            else:
                self.log.error_show(str(e))

    def send_cfg_cmd(self, address, val):
        try:
            _val = [val >> 16, val & 0x0000ffff]
            if self.connect_type == Modbus.Dobot:
                self.robot.robot_ctl.SetHoldRegs(self.master_index, address, 2, _val, "U16")
            else:
                self.master.execute(slave=self.gripper_ID,
                                    function_code=defines.WRITE_MULTIPLE_REGISTERS,
                                    starting_address=address,
                                    quantity_of_x=2,
                                    output_value=_val)
            time.sleep(0.1)
        except Exception as e:
            if self.log is None:
                print(str(e))
            else:
                self.log.error_show(str(e))

    def receive_ctl_cmd(self, address, read_nums=2):
        try:
            recv_data = self.master.execute(slave=self.gripper_ID,
                                            function_code=defines.READ_HOLDING_REGISTERS,
                                            starting_address=address,
                                            quantity_of_x=read_nums)
            time.sleep(0.1)
            return recv_data[0] << 16 & recv_data[1]
        except Exception as e:
            if self.log is None:
                print(str(e))
            else:
                self.log.error_show(str(e))

    def receive_state_cmd(self, address):
        try:
            recv_data = self.master.execute(slave=self.gripper_ID,
                                            function_code=defines.READ_COILS,
                                            starting_address=address)
            time.sleep(0.1)
            return recv_data
        except Exception as e:
            if self.log is None:
                print(str(e))
            else:
                self.log.error_show(str(e))

    def init_rm(self):
        self.send_cfg_cmd(5000, 3)
        self.send_cfg_cmd(5004, 80000)
        self.send_cfg_cmd(5006, 480000)
        self.send_cfg_cmd(5008, 480000)
        self.send_cfg_cmd(5010, 100)

        self.send_cfg_cmd(5020, 4)
        self.send_cfg_cmd(5032, 40000)
        self.send_cfg_cmd(5034, 40000)

        self.send_ctl_cmd(1452)

    def grasp(self, pos=0, force=15):
        if force is None:
            force = 15
        self.send_cfg_cmd(5032, force * 1000)
        self.send_ctl_cmd(1452)
        self.send_ctl_cmd(1409)

    def release(self, pos=0.04):
        if pos is None:
            pos = 0.04
        if pos <= 0.04:
            _pos = (0.04 - pos) * 1000000
        else:
            _pos = 0.04 * 1000000

        self.send_cfg_cmd(5002, int(_pos))
        self.send_ctl_cmd(1452)
        self.send_ctl_cmd(1408)


class SuckMotor:
    def __init__(self,
                 connect_type=Modbus.TCP_to_RTU,
                 ip="192.168.5.12", port=502,
                 rtu_port_name='COM3', baud_rate=115200,
                 dobot_robot: DobotControl = None,
                 log=None):
        self.gripper_ID = 0x03
        self.log = log
        self.connect_type = connect_type

        if connect_type == Modbus.RTU:
            self.master = modbus_rtu.RtuMaster(serial.Serial(port=rtu_port_name, baudrate=baud_rate))
            self.master.set_timeout(5.0)
            # self.master.set_verbose(True)
            self.master.open()

        elif connect_type == Modbus.TCP_to_RTU:
            self.master = modbus_tcp.TcpMaster(host=ip, port=port)
            self.master.set_timeout(5.0)
            # self.master.set_verbose(True)
            self.master.open()

        elif connect_type == Modbus.Dobot:
            if dobot_robot is None:
                dobot_robot = DobotControl()
            self.robot = dobot_robot
            self.robot.robot_ctl.SetTerminal485(baudRate=115200)
            self.master_index = self.robot.robot_ctl.ModbusCreate("127.0.0.1", 60000, self.gripper_ID, True)[0]
        else:
            self.log.error_show("Modbus Connect Type Error")

        self.init_motor()

    def send_cmd(self, address, val):
        try:
            if self.connect_type == Modbus.Dobot:
                self.robot.robot_ctl.SetHoldRegs(self.master_index, address, 1, [val], "U16")

            else:
                self.master.execute(slave=self.gripper_ID,
                                    function_code=defines.WRITE_SINGLE_REGISTER,
                                    starting_address=address,
                                    output_value=val)
        except Exception as e:
            if self.log is None:
                print(str(e))
            else:
                self.log.error_show(str(e))

    def receive_cmd(self, address, read_nums=1):
        try:
            if self.connect_type == Modbus.Dobot:
                data = self.robot.robot_ctl.GetHoldRegs(self.master_index, address, read_nums, "U16")
                if data is None:
                    return None
                recv_data = data[1][0]
            else:
                recv_data = self.master.execute(slave=self.gripper_ID,
                                                function_code=defines.READ_HOLDING_REGISTERS,
                                                starting_address=address,
                                                quantity_of_x=read_nums)
            return recv_data
        except Exception as e:
            if self.log is None:
                print(str(e))
            else:
                self.log.error_show(str(e))

    def init_motor(self):
        self.send_cmd(0x86, 4)
        self.send_cmd(0x86, 0)

        self.send_cmd(0x82, 100)
        self.send_cmd(0x83, 50)
        self.send_cmd(0x84, 100)

        self.send_cmd(0x80, 30712)

        while True:
            time.sleep(3)
            cur_vel = self.receive_cmd(0x102)
            if -1 <= cur_vel[0] <= 1:
                self.send_cmd(0x81, 0)
                break

        self.send_cmd(0x81, 1)
        self.send_cmd(0x82, 1000)
        self.send_cmd(0x83, 150)
        self.send_cmd(0x84, 1000)

        self.send_cmd(0x86, 4)
        self.send_cmd(0x86, 0)

        if self.log is not None:
            self.log.info_show("吸盘初始化完成")
        else:
            print("吸盘初始化完成")

    def forward_rotation(self, turn_nums=7):
        if turn_nums > 7:
            turn_nums = 7
        pos = turn_nums * 4095 + 2048
        self.send_cmd(0x80, pos)

    def reverse_rotation(self, turn_nums=7):
        if turn_nums > 8:
            turn_nums = 8
        pos = -(turn_nums * 4095) + 2048
        self.send_cmd(0x80, pos)


if __name__ == '__main__':
    # grasp_hand = UniversalGraspHandCtl()
    # grasp_hand.write_single_coil(0x0003, 0x0000)
    # grasp_hand.write_single_coil(0x00, 0x0000)
    # grasp_hand.write_holding_single_register(0x0006, 0)
    # while True:
    #     time.sleep(1)
    #     value = grasp_hand.read_input_registers(0x0003, 1)
    #     print(value)

    # grasp = DhModbus(connect_type=1)
    # grasp.grasp(pos=0.07, force=40)

    # grasp = RMHandModbus(connect_type=Modbus.RTU)
    # grasp.release(0.02)
    # grasp.grasp()

    # grasp = SuckMotor(connect_type=Modbus.RTU)
    # time.sleep(5)
    # grasp.forward_rotation(7)

    test = ModbusCtl(ip="192.168.1.88", port=502)

    test.write_holding_single_register(6200, 1)
    test.write_holding_single_register(6201, -1)
    test.write_holding_single_register(6202, 1)
