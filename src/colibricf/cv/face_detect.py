import cv2 as cv
import utils

capture = cv.VideoCapture(0)

def detectUpperBody(frame):
    haar_cascade_body = cv.CascadeClassifier("haar_cascade/haarcascade_upperbody.xml")
    matrix = haar_cascade_body.detectMultiScale(frame, scaleFactor=1.1, minNeighbors=3)
    return matrix

def detectFace(frame):
    haar_cascade_face = cv.CascadeClassifier("haar_cascade/haarcascade.xml")
    matrix = haar_cascade_face.detectMultiScale(frame, scaleFactor=1.4, minNeighbors=4)
    return matrix

def detectEye(frame):
    haar_cascade_eye = cv.CascadeClassifier("haar_cascade/haarcascade_eye.xml")
    matrix = haar_cascade_eye.detectMultiScale(frame, scaleFactor=2, minNeighbors=5)
    return matrix

while True:
    isTrue, frame = capture.read()
    
    frame = cv.flip(frame, 1)
    
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    face_rect = detectFace(gray)
    body_rect = detectUpperBody(gray)
    eye_rect = detectEye(gray)

    for (x, y, w, h) in face_rect:
        cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), thickness=2)
        cv.putText(frame, f'face ({x},{y})', (x , y - 20), cv.FONT_HERSHEY_PLAIN, 1.1, (0, 255, 0), 2)
    
    for (x, y, w, h) in body_rect:
        cv.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), thickness=2)
        cv.putText(frame, f'body ({x},{y})', (x , y - 20), cv.FONT_HERSHEY_PLAIN, 1.1, (0, 0, 255), 2)
    
    for (x, y, w, h) in eye_rect:
        cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), thickness=2)
        cv.putText(frame, f'eye ({x},{y})', (x , y - 20), cv.FONT_HERSHEY_PLAIN, 1.1, (255, 0, 0), 2)       

    resized = utils.resize(frame)
    cv.imshow("Video", resized)
    
    if cv.waitKey(20) & 0xFF == ord("x"):
        break
    
capture.release()
cv.destroyAllWindows()
    
