from ultralytics import YOLO
import cv2
'''
cam = cv2.VideoCapture(4)

while True:
    check, frame = cam.read()

    cv2.imshow('video', frame)

    key = cv2.waitKey(1)
    if key == 27:
        break

cam.release()
cv2.destroyAllWindows()
'''
img = cv2.imread('frame212_jpg.rf.8151142b06df60232bb21ed40faacf21.jpg', cv2.IMREAD_COLOR)
cv2.imshow('img', img)
cv2.waitKey(1000)
cv2.destroyAllWindows()

model= YOLO('/home/pearl/yolov8/runs/detect/train6/weights/best.pt')

results = model(source=img, show=False, conf=0.4, save=False)
cv2.imshow("yolo", results[0].plot())
cv2.waitKey(1000)
cv2.destroyAllWindows()
for r in results:
    for b in r.boxes:
        print(r.names[int(b.cls)])
        print(b.xyxy[0])

