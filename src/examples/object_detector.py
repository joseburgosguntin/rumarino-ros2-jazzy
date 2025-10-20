import cv2
import torch

model = torch.hub.load("ultralytics/yolov5:v6.0", "yolov5n")  # nano version


def object_detector():
    capture = cv2.VideoCapture(0)
    if not capture:
        return

    frame_count = 0
    while True:
        ret, frame = capture.read()
        if not ret:
            break

        if frame_count % 3 == 0:
            continue

        results = model(frame)

        df = results.pandas().xyxy[0]

        # Loop for bounding boxes
        for i in range(len(df)):
            xmin, ymin, xmax, ymax = (
                int(df.iloc[i, 0]),
                int(df.iloc[i, 1]),
                int(df.iloc[i, 2]),
                int(df.iloc[i, 3]),
            )
            confidence = df.iloc[i, 4]
            class_name = df.iloc[i, 6]

            label = f"{class_name} {confidence:.2f}"
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            cv2.putText(
                frame,
                label,
                (xmin, ymin - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

        # Display detections
        cv2.imshow("YOLOv5 Object Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    capture.release()
    cv2.destroyAllWindows()


object_detector()
