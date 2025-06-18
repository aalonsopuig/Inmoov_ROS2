#!/usr/bin/env python3
"""
generate_encodings.py

This script generates facial embeddings for known individuals based on images stored in
the `faces_db/` folder (located one level up from this script). Each subfolder in `faces_db/`
corresponds to one person and contains a set of face images. The script uses Dlib's models
to extract 128-dimensional face embeddings for each detected face. These encodings are
stored in a serialized `encodings.pickle` file, which can later be used for real-time
face recognition.

Author: Alejandro Alonso Puig
GitHub: https://github.com/aalonsopuig
Date: June 3, 2025
License: Apache 2.0
"""

# --- Standard libraries ---
import os                  # Filesystem navigation
import cv2                 # OpenCV for image loading and processing
import dlib                # Dlib for face detection and embeddings
import pickle              # Serialization for saving embeddings

# --- Paths configuration (always absolute for robustness) ---
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FACES_DB_DIR = os.path.abspath(os.path.join(BASE_DIR, '..', 'faces_db'))    # Faces database location
MODELS_DIR = os.path.join(BASE_DIR, 'models')                               # Model files location
OUTPUT_FILE = os.path.join(BASE_DIR, 'data', 'encodings.pickle')            # Output .pickle file

# --- Load Dlib models ---
# - shape_predictor_68_face_landmarks.dat: detects 68 facial landmarks per face
# - dlib_face_recognition_resnet_model_v1.dat: outputs 128D embedding per face
face_detector = dlib.get_frontal_face_detector()
shape_predictor = dlib.shape_predictor(os.path.join(MODELS_DIR, 'shape_predictor_68_face_landmarks.dat'))
face_recognizer = dlib.face_recognition_model_v1(os.path.join(MODELS_DIR, 'dlib_face_recognition_resnet_model_v1.dat'))

# --- Dictionary to store the encodings for each person ---
encodings = {}

# --- Loop through each person's folder ---
for person_name in os.listdir(FACES_DB_DIR):
    person_path = os.path.join(FACES_DB_DIR, person_name)
    if not os.path.isdir(person_path):
        continue  # Only process directories

    encodings[person_name] = []

    # --- Process each image in the person's folder ---
    for img_name in os.listdir(person_path):
        img_path = os.path.join(person_path, img_name)
        image = cv2.imread(img_path)

        if image is None:
            print(f"[WARN] Could not read image {img_path}")
            continue

        # Convert image from BGR (OpenCV) to RGB (Dlib expects RGB)
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Detect faces in the image
        detections = face_detector(rgb_image)

        if len(detections) == 0:
            print(f"[WARN] No face detected in {img_path}")
            continue

        # Use the first detected face (recommended: only one per image)
        face = detections[0]

        # Detect facial landmarks and compute embedding (128D)
        shape = shape_predictor(rgb_image, face)
        descriptor = face_recognizer.compute_face_descriptor(rgb_image, shape)

        # Convert tuple to list for serialization
        encodings[person_name].append(list(descriptor))

        print(f"[INFO] Processed {img_name} for {person_name}")

# --- Save the dictionary to a .pickle file for later use in recognition ---
with open(OUTPUT_FILE, 'wb') as f:
    pickle.dump(encodings, f)

print(f"[INFO] Encodings saved to {OUTPUT_FILE}")
