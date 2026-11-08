import sounddevice as sd
import numpy as np
import vosk
import json
import time
import psutil
import csv
import os
from gtts import gTTS
from datetime import datetime

# =========================
# Configuration
# =========================
MODEL_PATH = "../models/vosk-model-small-en-us-0.15"
SAMPLE_RATE = 16000
DURATION = 4  # seconds
CSV_FILE = "experiment1_metrics.tsv"   # <- TSV FILE

# =========================
# Load Vosk Model
# =========================
print("\nLoading Vosk model...")
model = vosk.Model(MODEL_PATH)
recognizer = vosk.KaldiRecognizer(model, SAMPLE_RATE)

print("Model loaded. Press Ctrl+C to stop.")
input("Press ENTER to start...")

# =========================
# Experiment Loop
# =========================
iteration = 1

try:
    while True:
        print(f"\nTEST {iteration} - Speak now")

        start_total = time.time()
        cpu_before = psutil.cpu_percent(interval=None)

        # --- Record audio ---
        start_stt = time.time()
        audio = sd.rec(
            int(DURATION * SAMPLE_RATE),
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype="int16"
        )
        sd.wait()

        recognizer.AcceptWaveform(audio.tobytes())
        result = json.loads(recognizer.Result())
        recognized_text = result.get("text", "")
        stt_latency = time.time() - start_stt

        # --- Response ---
        if recognized_text.strip() == "":
            response_text = "Sorry, I did not understand"
            success_flag = 0
        else:
            response_text = f"You said {recognized_text}"
            success_flag = 1

        start_tts = time.time()
        gTTS(response_text, lang="en").save("response.mp3")
        os.system("mpg123 response.mp3")
        tts_latency = time.time() - start_tts

        cpu_usage = psutil.cpu_percent(interval=None)
        total_latency = time.time() - start_total

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        header = [
            "no",
            "timestamp",
            "recognized_text",
            "stt_latency_sec",
            "tts_latency_sec",
            "total_latency_sec",
            "cpu_usage_percent",
            "success_flag"
        ]

        row = [
            iteration,
            timestamp,
            recognized_text,
            round(stt_latency, 3),
            round(tts_latency, 3),
            round(total_latency, 3),
            round(cpu_usage, 2),
            success_flag
        ]

        file_exists = os.path.isfile(CSV_FILE)

        with open(CSV_FILE, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f, delimiter="\t")   # ← TAB DELIMITER
            if not file_exists:
                writer.writerow(header)
            writer.writerow(row)

        print("Saved:", row)
        iteration += 1

except KeyboardInterrupt:
    print("\nExperiment stopped safely.")
