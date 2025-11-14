import os
import rospy
from enum import Enum
from clover import srv
from datetime import datetime

class Extension(Enum):
    VIDEO_MP4 = '.mp4'
    IMG_PNG = '.png'
    IMG_JPEG = '.jpg'
    TASK_LOG = '.log'
    SYS_LOG = '.csv'

class FileManager():
    def __init__(self, datadir: str = '/var/lib/clover/'):
        self.datadir = datadir

    def filename(self, extension: Extension) -> str:
        path = None
        if extension == Extension.VIDEO_MP4:
            path = os.path.join(self.datadir, 'video')
        elif extension == Extension.IMG_JPEG or extension == Extension.IMG_PNG:
            path = os.path.join(self.datadir, 'image')
        elif extension == Extension.TASK_LOG:
            path = os.path.join(self.datadir, 'log', 'mission')
        elif extension == Extension.SYS_LOG:
            path = os.path.join(self.datadir, 'log', 'system')

        self.mkdir(path)

        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        system = 'clover'
        filename = os.path.join(path, f'{system}-{timestamp}{extension.value}')

        return filename

    def mkdir(self, dirname: str) -> None:
        if not os.path.exists(dirname):
            try:
                os.makedirs(dirname, exist_ok=True)
            except Exception as e:
                rospy.logerr(e)


    def add_metadata(self, filename: str):
        telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)(frame_id='body')

        lat = telemetry.lat
        lon = telemetry.lon
        alt = telemetry.alt

        def to_dms(value: float):
            abs_degrees = abs(value)
            degress = int(abs_degrees)
            minutes = int((abs_degrees - degress) * 60)
            seconds = int(((abs_degrees - degress) * 60 - minutes) * 60 * 10000)

            return ((degress, 1), (minutes, 1), (seconds, 10000))

        exif_dict = {
            "GPS": {
                piexif.GPSIFD.GPSLatitudeRef: b"N" if lat >= 0 else b"S",
                piexif.GPSIFD.GPSLatitude: to_dms(lat),
                piexif.GPSIFD.GPSLongitudeRef: b"E" if lon >= 0 else b"W",
                piexif.GPSIFD.GPSLongitude: to_dms(lon),
                piexif.GPSIFD.GPSAltitude: (int(alt * 1000), 1000),
                piexif.GPSIFD.GPSAltitudeRef: 0,
            }
        }

        exif_bytes = piexif.dump(exif_dict)
        piexif.insert(exif_bytes, filename)

