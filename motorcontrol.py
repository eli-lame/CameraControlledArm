import cv2
import mediapipe as mp
import serial
import time

# Open serial connection to Arduino (adjust COM port if needed)
arduino = serial.Serial('COM3', 9600)
time.sleep(2)  # wait for Arduino to reset

# Initialize MediaPipe Hands and drawing utilities
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

# Start webcam capture
cap = cv2.VideoCapture(0)

# X-position thresholds for left/stop/right commands
THRESHOLD_LOW = 0.6
THRESHOLD_HIGH = 0.8

# Create MediaPipe Hands instance
# Only track 1 hand, and use moderate confidence levels
with mp_hands.Hands(max_num_hands=1,
                    min_detection_confidence=0.7,
                    min_tracking_confidence=0.7) as hands:

    while True:
        # Read a frame from the camera
        success, frame = cap.read()
        if not success:
            break

        # Convert image BGR -> RGB because MediaPipe needs RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Run hand landmark detection
        results = hands.process(frame_rgb)

        # If at least one hand is detected
        if results.multi_hand_landmarks:
            hand = results.multi_hand_landmarks[0]   # get first hand

            # Draw the hand landmarks on the image
            mp_draw.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

            # Get the x-position of the index fingertip (landmark 8)
            x = hand.landmark[8].x

            # Compare fingertip x-position to thresholds
            if x < THRESHOLD_LOW:
                arduino.write(b"L\n")   # send LEFT command to Arduino
                print("LEFT")

            elif x > THRESHOLD_HIGH:
                arduino.write(b"R\n")   # send RIGHT command to Arduino
                print("RIGHT")

            else:
                arduino.write(b"S\n")   # send STOP command
                print("STOP")

        # Display the camera feed with landmarks
        cv2.imshow("Hand Control", frame)

        # Exit on ESC key
        if cv2.waitKey(1) & 0xFF == 27:
            break

# Cleanup: release the camera and close windows
cap.release()
cv2.destroyAllWindows()
