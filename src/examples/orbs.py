import cv2
import numpy as np

# Initialize the webcam
cap = cv2.VideoCapture(0)

# Initialize the ORB detector
orb = cv2.ORB_create()

# Define FLANN-based matcher parameters
FLANN_INDEX_LSH = 6
index_params = dict(
    algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1
)
search_params = dict(checks=50)
flann = cv2.FlannBasedMatcher(index_params, search_params)

# Grab a frame from the webcam to use as a template
while True:
    ret, template_frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Display the template and wait for keypress to confirm
    cv2.imshow("Template", template_frame)
    if cv2.waitKey(1) & 0xFF == ord("t"):
        break

# Convert the template frame to grayscale
template_gray = cv2.cvtColor(template_frame, cv2.COLOR_BGR2GRAY)

# Detect keypoints and descriptors in the template
template_keypoints, template_descriptors = orb.detectAndCompute(template_gray, None)

while True:
    # Capture frame-by-frame from the webcam
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Convert the current frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect keypoints and descriptors in the current frame
    frame_keypoints, frame_descriptors = orb.detectAndCompute(gray_frame, None)

    # Match descriptors using FLANN-based matcher
    if frame_descriptors is not None and template_descriptors is not None:
        matches = flann.knnMatch(template_descriptors, frame_descriptors, k=2)

        # Filter good matches using Lowe's ratio test
        good_matches = []
        for match in matches:
            if len(match) == 2:
                m, n = match
                if m.distance < 0.75 * n.distance:
                    good_matches.append(m)

        # Confidence level calculation based on good matches
        total_keypoints = len(template_keypoints)  # Total keypoints in the template
        num_good_matches = len(good_matches)  # Number of good matches

        # Calculate confidence level as a percentage
        confidence_level = (
            (num_good_matches / total_keypoints) * 100 if total_keypoints > 0 else 0
        )

        # Display the confidence level on the frame
        cv2.putText(
            frame,
            f"Confidence: {confidence_level:.2f}%",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )

        # Draw matches between the template and the live frame
        matched_frame = cv2.drawMatches(
            template_frame,
            template_keypoints,
            frame,
            frame_keypoints,
            good_matches,
            None,
            flags=2,
        )

        # Display the frame with matches
        cv2.imshow("Matches", matched_frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release the webcam and close windows
cap.release()
cv2.destroyAllWindows()
