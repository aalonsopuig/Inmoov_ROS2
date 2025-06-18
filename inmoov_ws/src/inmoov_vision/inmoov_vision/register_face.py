#!/usr/bin/env python3
#
# File: register_face.py
# Description: Utility script to capture face images from webcam and store them
#              for later recognition. Saves N grayscale images per person.
# Author: Alejandro Alonso Puig
# GitHub: https://github.com/aalonsopuig
# License: Apache License 2.0
# Date: June 2025
#

import cv2
import os
import time

# === Configuration ===
NUM_IMAGES = 5  # Number of face images to capture
WAIT_TIME = 4.0  # Seconds to wait between captures
HAAR_PATH = os.path.join(os.path.dirname(__file__), 'haarcascades', 'haarcascade_frontalface_default.xml')
DB_DIR = os.path.join(os.path.dirname(__file__), '..', 'faces_db')  # Database folder

def main():
    # Load Haar Cascade classifier
    face_cascade = cv2.CascadeClassifier(HAAR_PATH)
    if face_cascade.empty():
        print("[ERROR] Haar cascade file could not be loaded.")
        return

    # Open webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Could not open webcam.")
        return

    # Get person name
    name = input("Enter the name of the person to register: ").strip()
    if not name:
        print("Invalid name.")
        return

    # Create folder for the person
    person_dir = os.path.join(DB_DIR, name)
    os.makedirs(person_dir, exist_ok=True)

    print(f"[INFO] Capturing {NUM_IMAGES} face images for '{name}'...")

    count = 0
    while count < NUM_IMAGES:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Could not read frame.")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.2, 5)

        for (x, y, w, h) in faces:
            # Draw rectangle
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Save cropped face
            face_img = gray[y:y + h, x:x + w]
            filename = os.path.join(person_dir, f'{name}_{count+1}.png')
            cv2.imwrite(filename, face_img)
            print(f"[INFO] Saved: {filename}")
            count += 1
            time.sleep(WAIT_TIME)
            break  # Capture one face per frame

        # Show feedback window
        cv2.imshow("Face Registration", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Interrupted by user.")
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"[INFO] Face registration complete for '{name}'.")

if __name__ == "__main__":
    main()
