#!/usr/bin/env python3
"""
test_tts.py
Test script for text-to-speech using pyttsx3 with eSpeak NG in Spanish.
Authors: Alejandro Alonso Puig + ChatGPT 4.1
GitHub: https://github.com/aalonsopuig
Date: June 2025
License: Apache 2.0
"""

import pyttsx3

engine = pyttsx3.init()


engine.setProperty('voice', "Spanish (Spain)")


#engine.setProperty('rate', 150)  # Puedes ajustar la velocidad si lo deseas

text = "Hola, soy Paul, tu robot InMoov. Esto es una prueba de síntesis de voz en español."

engine.say(text)
engine.runAndWait()

