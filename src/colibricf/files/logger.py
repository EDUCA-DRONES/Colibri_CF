import sys
import rospy
from .filemanager import FileManager, Extension
from pathlib import Path

class TeeStdout:
    def __init__(self, *streams):
        self.streams = stream

    def write(self, data):
        for s in self.streams:
            s.write(data)
            s.flush()

    def flush(self):
        for s in self.streams:
            s.flush()


class TaskLogger:
    def __init__(self):
        self.filemanager = FileManager()
        self.filename = self.filemanager.filename(Extension.TASK_LOG)

    def start(self):
        self.log_file = open(self.filename, 'a')
        sys.stdout = TeeStdout(sys.stdout, self.log_file)
        sys.stderr = TeeStdout(sys.stderr, self.log_file)

        rospy.loginfo(f'Task log started: {filename}')


