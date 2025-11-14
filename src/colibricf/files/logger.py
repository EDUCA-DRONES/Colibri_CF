import rospy
from .filemanager import FileManager, Extension
from rosgraph_msgs.msg import Log

class Logger:
    def __init__(self):
        self.filemanager = FileManager()
        self.filename = self.filemanager.filename(Extension.TASK_LOG)

    def start(self):
        def _callback(msg):
            level_map = {
                msg.DEBUG: 'DEBUG',
                msg.INFO: 'INFO',
                msg.WARN: 'WARN',
                msg.ERROR: 'ERROR',
                msg.FATAL: 'FATAL'
            }

            level = level_map.get(msg.level, 'UNKNOWN')
            timestamp = msg.header.stamp.to_sec()

            line = f'[{level}] [{timestamp:.5f}]: {msg.msg}\n'
            self.log_file.write(line)
            self.log_file.flush()

        self.log_file = open(self.filename, 'a')
        self.rosout = rospy.Subscriber('/rosout', Log, _callback)
        rospy.loginfo(f'Task log started: {self.filename}')

    def stop(self):
        try:
            if self.file and self.rosout is not None:
                self.rosout.unregister()
                self.log_file.flush()
                self.log_file.close()

        except Exception as e:
            rospy.logerr()
