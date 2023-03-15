from __future__ import annotations
import time
from typing import List
import genpy
import rospy
from avl_fault_detection import util
from pathlib import Path


class AvlLogger:

    _AVL_LOG_DIR = Path('/var/avl_logs/current/log')

    def __init__(self, name: str):
        self.name = name
        self.log_path = Path(AvlLogger._AVL_LOG_DIR, f"{name}.log")
        self.log_file = None

    def write_msg_header(self, msg: genpy.Message, units: List[str] | None = None):
        if not units:
            units = msg._slot_types
        self.write_header(msg.__slots__, units)
    
    def write_msg_data(self, msg: genpy.Message):
        self.write_data([getattr(msg, slot) for slot in msg.__slots__])

    def write_header(self, names: List[str], units: List[str]):
        if self.log_file:
            rospy.logwarn_once("You probably called write header twice!")
        names = ["time_s"] + names
        units = ["none"] + units
        self.log_file = open(self.log_path, 'w')
        self.log_file.write(' '.join(names) + '\n')
        self.log_file.write(' '.join(units) + '\n')

    def write_data(self, data: List):
        if not self.log_file:
            rospy.logerr("You must call write_header before calling write_data")
        else:
            data = [time.time()] + data
            self.log_file.write(' '.join([str(d) for d in data]) + '\n')

    def flush(self):
        if self.log_file:
            self.log_file.flush()

    def close(self):
        if self.log_file:
            self.log_file.close()