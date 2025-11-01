import cv2 as cv

def resize(frame, scale=1):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)

    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)

def changeRes(capture, width, height):
    capture.set(3, width)
    capture.set(4, height)
