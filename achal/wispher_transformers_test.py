from transformers.pipelines.audio_utils import ffmpeg_microphone_live
import time
from transformers import pipeline
import torch
import sys

device = "cuda:0" if torch.cuda.is_available() else "cpu"



transcriber = pipeline("automatic-speech-recognition", 
                           model="openai/whisper-base.en", 
                           device=device)


def transcribe(chunk_length_s=5.0, stream_chunk_s=1.0):
    sampling_rate = transcriber.feature_extractor.sampling_rate

    mic = ffmpeg_microphone_live(
        sampling_rate=sampling_rate,
        chunk_length_s=chunk_length_s,
        stream_chunk_s=stream_chunk_s,
    )

    print("Start speaking...")
    for item in transcriber(mic, generate_kwargs={"max_new_tokens": 128}):
        sys.stdout.write("\033[K")
        print(item["text"], end="\r")
        if not item["partial"][0]:
            break

    return item["text"]

transcribe()
