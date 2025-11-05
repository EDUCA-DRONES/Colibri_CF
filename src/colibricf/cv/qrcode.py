from clover import long_callback
from cv_bridge import CvBridge
from pyzbar import pyzbar

@long_callback
def _qrcode_callback(data):
    '''
    Read a qrcode
    '''

    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, 'bgr8')
    qrcodes = pyzbar.decode(frame)
    for qrcode in qrcodes:
        b_data = qrcode.data.decode('utf-8')
        b_type = qrcode.type
        (x, y, w, h) = qrcode.rect
        xc = x + w/2
        yc = y + h/2
        print(f'Found {b_type} with data {b_data} with center at x={xc}, y={yc}')
