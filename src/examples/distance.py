import cv2
from ultralytics import YOLO

model = YOLO("yolo11n.pt")
cap = cv2.VideoCapture(0)
bottle = 39
bottle_height = 0.667
focal_length = 652.17
while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)
    bottle_detections = [det for det in results[0].boxes if int(det.cls[0]) == bottle]

    for det in bottle_detections:
        box = det.xyxy[0].cpu().numpy().astype(int)
        x1, y1, x2, y2 = box
        pixel_height = y2 - y1
        if pixel_height > 0:
            distance_feet = (bottle_height * focal_length) / pixel_height
            distance_feet = round(distance_feet, 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            conf = det.conf[0].cpu().numpy()
            label = f"Bottle: {conf:.2f}, Distance: {distance_feet} ft"
            cv2.putText(
                frame,
                label,
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

    cv2.imshow("YOLO Water Bottle Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
