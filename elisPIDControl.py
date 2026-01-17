import cv2
import mediapipe as mp
import serial
import time

# Load MediaPipe hand tracking and drawing utilities
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

#115200 baud rate is common for high speeds
arduino = serial.Serial('COM3', 115200)

#timer to slow down the video capture rate (arduino cant handle super fast rates)
lastSend = 0
sendInterval = 0.01  # seconds (30 ms ≈ 33 Hz)

# Start webcam capture (0 = default webcam)
cap = cv2.VideoCapture(0)

# Initialize MediaPipe Hands with support for tracking 2 hands
with mp_hands.Hands(
        max_num_hands=2,                 # Track up to 2 hands
        min_detection_confidence=0.7,    # Confidence threshold for detecting a hand
        min_tracking_confidence=0.7      # Confidence threshold for tracking landmarks
    ) as hands:

    while True:
        # Read a frame from the webcam
        success, frame = cap.read()
        if not success:
            break  # Stop if the webcam fails or is disconnected

        # Convert the BGR image (OpenCV default) to RGB for MediaPipe
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Run hand tracking on the frame
        results = hands.process(frame_rgb)

        # If hands were detected in the frame
        if results.multi_hand_landmarks:
            # Loop through each detected hand and its handedness info
            for hand_landmarks, handedness in zip(
                results.multi_hand_landmarks,
                results.multi_handedness
            ):
                # Draw the hand skeleton/landmarks on the video frame
                mp_draw.draw_landmarks(
                    frame,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS
                )

                # Determine whether this hand is Left or Right
                label = handedness.classification[0].label

                # Get the position of the index fingertip (landmark 8)
                index_tip = hand_landmarks.landmark[8]
                x, y = index_tip.x, index_tip.y  # Normalized coordinates (0–1)

                #this is the encoder position that will be sent to the motor
                motorPos = int(x * 500)

                # Print the fingertip coordinates for debugging/processing
                print(f"{label} hand index tip: x={x:.3f}, y={y:.3f}")
                print(motorPos)

                #second part of the upload rate timer for arduino serial 
                now = time.time()
                if now - lastSend >= sendInterval:
                    arduino.write(f"{motorPos}\n".encode())
                    lastSend = now




        # Display the video feed with drawn landmarks
        cv2.imshow("Two-Hand Tracking", frame)

        # Exit if the Escape key (Esc) is pressed
        if cv2.waitKey(1) & 0xFF == 27:
            break

# Release the webcam and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
