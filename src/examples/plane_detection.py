import cv2
import numpy as np


def plane_detection():
    vid = cv2.VideoCapture(0)

    if not vid.isOpened():
        print("Error: Couldn't open video.")
        return

    while True:
        ret, frame = vid.read()

        if not ret:
            print("Error: Couldn't read frame.")
            break

        # Calculate frame height
        frame_height = frame.shape[0]

        # Convert to grayscale and apply GaussianBlur for plane detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # Edge detection
        edges = cv2.Canny(gray, 50, 150)

        # Detect lines
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, threshold=50, minLineLength=100, maxLineGap=10
        )

        # Draw detected lines on the frame
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]

                # Classify lines based on their slope and position
                slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else float("inf")
                angle = np.arctan(slope) * 180 / np.pi  # Convert to degrees

                # Check line position
                if abs(angle) < 10 and y1 > frame_height * 0.6:  # Floors
                    cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Red lines
                    cv2.putText(
                        frame,
                        "Floor",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 0, 255),
                        2,
                    )
                elif abs(angle) > 80 and y1 < frame_height * 0.6:  # Walls
                    cv2.line(
                        frame, (x1, y1), (x2, y2), (0, 255, 255), 2
                    )  # Yellow lines
                    cv2.putText(
                        frame,
                        "Wall",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255),
                        2,
                    )

        cv2.imshow("Plane Detection", frame)

        # Exit by pressing 'q' key
        if cv2.waitKey(5) & 0xFF == ord("q"):
            break

    # Release video and close all windows
    vid.release()
    cv2.destroyAllWindows()


plane_detection()
