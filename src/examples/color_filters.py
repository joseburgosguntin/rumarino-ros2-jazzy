import cv2
import numpy as np


def color_filters(tolerance=10, min_area=500, min_confidence=30):
    vid = cv2.VideoCapture(0)

    if not vid.isOpened():
        print("Error: Couldn't open video.")
        return

    print(
        "Press: \n"
        "'t' to increase tolerance\n"
        "'y' to decrease tolerance\n"
        "'a' to increase minimum area\n"
        "'s' to decrease minimum area\n"
        "'c' to increase minimum confidence\n"
        "'v' to decrease minimum confidence\n"
        "'q' to quit."
    )

    while True:
        ret, frame = vid.read()

        if not ret:
            print("Error: Couldn't read frame.")
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (25, 25), 0)

        # Masks for color red
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10 + tolerance, 255, 255])
        lower_red2 = np.array([170 - tolerance, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask_red1 | mask_red2

        # Reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        filtered_frame = cv2.bitwise_and(frame, frame, mask=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        object_count = 0

        # Bounding boxes
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_area:
                object_count += 1
                x, y, w, h = cv2.boundingRect(contour)

                # Confidence level
                contour_mask = np.zeros(mask.shape, np.uint8)
                cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)

                red_pixels_in_contour = cv2.bitwise_and(mask, mask, mask=contour_mask)
                red_pixel_count = np.sum(red_pixels_in_contour == 255)
                total_pixels_in_contour = w * h

                # To avoid dividing by 0
                if total_pixels_in_contour > 0:
                    confidence = (red_pixel_count / total_pixels_in_contour) * 100
                else:
                    confidence = 0

                # To avoid a percentage higher than 100%
                if confidence > 100:
                    confidence = 100

                # Ignoring objects with low confidence levels
                if confidence > min_confidence:
                    cv2.rectangle(
                        filtered_frame, (x, y), (x + w, y + h), (0, 255, 0), 2
                    )
                    cv2.putText(
                        filtered_frame,
                        f"Conf: {confidence:.2f}%",
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )

        # Number of objects detected
        cv2.putText(
            filtered_frame,
            f"Objects: {object_count}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

        # Display filtered frame
        cv2.imshow("Filtered frame", filtered_frame)

        # User input
        key = cv2.waitKey(5) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("t"):
            tolerance += 1
            print(f"Tolerance increased to {tolerance}.")
        elif key == ord("y"):
            tolerance = max(0, tolerance - 1)
            print(f"Tolerance decreased to {tolerance}.")
        elif key == ord("a"):
            min_area += 10
            print(f"Minimum area increased to {min_area}.")
        elif key == ord("s"):
            min_area = max(0, min_area - 10)
            print(f"Minimum area decreased to {min_area}.")
        elif key == ord("c"):
            min_confidence += 5
            print(f"Minimum confidence increased to {min_confidence}.")
        elif key == ord("v"):
            min_confidence = max(0, min_confidence - 5)
            print(f"Minimum confidence decreased to {min_confidence}.")

    # Release video and close all windows
    vid.release()
    cv2.destroyAllWindows()


color_filters()
