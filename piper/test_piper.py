import subprocess

text = "Hola, soy Paul, tu robot InMoov. Prueba de voz con Piper."
cmd = [
    "./piper",
    "--model", "es_ES-davefx-medium.onnx",
    "--config", "es_ES-davefx-medium.onnx.json",
    "--output_file", "output.wav"
]

proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
proc.communicate(input=text.encode())
subprocess.run(["aplay", "output.wav"])
