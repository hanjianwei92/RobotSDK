import logging
from logging import handlers
from queue import Queue
import datetime
from pathlib import Path
from inspect import currentframe, stack, getmodule, getframeinfo


class CasLogger(object):
    level_relations = {
        'debug': logging.DEBUG,
        'info': logging.INFO,
        'warning': logging.WARNING,
        'error': logging.ERROR,
        'crit': logging.CRITICAL
    }  # 日志级别关系映射

    def __init__(self,
                 filename,
                 level='info',
                 when='D',
                 backCount=3,
                 fmt='%(levelname)s - %(asctime)s - %(message)s',
                 info_queue: Queue = None,
                 error_queue: Queue = None):
        self.info_queue = info_queue
        self.error_queue = error_queue
        self.logger = logging.getLogger(filename)
        self.logger.propagate = False
        if len(self.logger.handlers) == 0:
            format_str = logging.Formatter(fmt)  # 设置日志格式
            self.logger.setLevel(self.level_relations.get(level))  # 设置日志级别
            sh = logging.StreamHandler()  # 往屏幕上输出
            sh.setFormatter(format_str)  # 设置屏幕上显示的格式
            Path(filename).parent.mkdir(exist_ok=True, parents=True)
            th = handlers.TimedRotatingFileHandler(filename=filename, when=when, backupCount=backCount,
                                                   encoding='utf-8')  # 往文件里写入#指定间隔时间自动生成文件的处理器
            # 实例化TimedRotatingFileHandler
            # interval是时间间隔，backupCount是备份文件的个数，如果超过这个个数，就会自动删除，when是间隔的时间单位，单位有以下几种：
            # S 秒
            # M 分
            # H 小时、
            # D 天、
            # W 每星期（interval==0时代表星期一）
            # midnight 每天凌晨
            th.setFormatter(format_str)  # 设置文件里写入的格式
            self.logger.addHandler(sh)  # 把对象加到logger里
            self.logger.addHandler(th)

    # 白色  # FFFFFF;红色 #FF0000;绿色 #00FF00;蓝色 #0000FF;牡丹红 #FF00FF; 青色 #00FFFF;黄色 #FFFF00;黑色 #000000;
    def info_show(self, info):
        previous_frame = currentframe().f_back
        frame_info = getframeinfo(previous_frame)
        file_name = frame_info[0]
        line = frame_info[1]
        self.logger.info(f"{file_name} - {line} : {info}")
        cur_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-4]
        if self.info_queue is not None:
            self.info_queue.put("<font color=\"#000000\">" + str(cur_time) + ": " + info + "</font>")

    def error_show(self, error: str):
        previous_frame = currentframe().f_back
        frame_info = getframeinfo(previous_frame)
        file_name = frame_info[0]
        line = frame_info[1]
        self.logger.error(f"{file_name} - {line} : {error}")
        cur_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-4]
        if self.error_queue is not None:
            self.error_queue.put("<font color=\"#FF0000\">" + "【ERROR】" + str(cur_time) + ": " + error + "</font>")

    def finish_show(self, info: str):
        previous_frame = currentframe().f_back
        frame_info = getframeinfo(previous_frame)
        file_name = frame_info[0]
        line = frame_info[1]
        self.logger.critical(f"{file_name} - {line} : {info}")
        cur_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-4]
        if self.info_queue is not None:
            self.info_queue.put("<font color=\"#006600\">" + "【FINISH】" + str(cur_time) + ": " + info + "</font>")

    def warn_show(self, warn: str):
        previous_frame = currentframe().f_back
        frame_info = getframeinfo(previous_frame)
        file_name = frame_info[0]
        line = frame_info[1]
        self.logger.critical(f"{file_name} - {line} : {warn}")
        cur_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-4]
        if self.info_queue is not None:
            self.info_queue.put("<font color=\"#ff6600\">" + "【WARNING】" + str(cur_time) + ": " + warn + "</font>")


if __name__ == '__main__':
    log = CasLogger('../all.log', level='debug')
    log.logger.debug('debug')
    log.logger.info('info')
    log.logger.warning('警告')
    log.logger.error('报错')
    log.logger.critical('严重')
    CasLogger('../error.log', level='error').logger.error('error')
