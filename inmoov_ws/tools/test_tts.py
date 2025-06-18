import torch
from TTS.api import TTS
import sounddevice as sd
import numpy as np

tts = TTS(model_name="tts_models/es/css10/vits", progress_bar=False, gpu=False)
#tts = TTS(model_name="tts_models/es/mai/tacotron2-DDC", progress_bar=False, gpu=False)
texto = "Hola, soy InMoov. Ahora utilizo una voz mucho más natural y clara que antes."
audio_arr = tts.tts(text=texto)
sample_rate = tts.synthesizer.output_sample_rate
audio_np = np.array(audio_arr, dtype=np.float32)
sd.play(audio_np, samplerate=sample_rate)
sd.wait()
