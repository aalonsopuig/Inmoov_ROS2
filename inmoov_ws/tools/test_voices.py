import pyttsx3

# Inicializa el motor
engine = pyttsx3.init()

# Lista todas las voces disponibles
voices = engine.getProperty('voices')

print("Voces disponibles en tu sistema:")
for i, voice in enumerate(voices):
    # Imprime información de cada voz
    print(f"[{i}] id: {voice.id}")
    print(f"    name: {voice.name}")
    print(f"    languages: {voice.languages}")
    print(f"    gender: {voice.gender}")
    print(f"    age: {voice.age}")
    print()

print("---- Probando voces en español ----")

# Frase de prueba
test_text = "Hola, soy Paul, tu robot InMoov. Esto es una prueba de voz."

# Prueba todas las voces que sean en español
for voice in voices:
    # Busca si la voz es española (esto depende de la instalación y sistema)
    if any('es' in lang.decode('utf-8') if isinstance(lang, bytes) else 'es' in str(lang) for lang in voice.languages):
        print(f"Probando voz: {voice.name} (id: {voice.id})")
        engine.setProperty('voice', voice.id)
        engine.say(test_text)
        engine.runAndWait()

# Opcional: cómo seleccionar una voz explícita
# (sustituye 'espeak-ng+mb-es1' por el id concreto que imprima tu sistema)
# engine.setProperty('voice', 'espeak-ng+mb-es1')
# engine.say("Esta es la voz mbrola española número 1")
# engine.runAndWait()
