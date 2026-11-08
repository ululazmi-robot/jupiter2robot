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
DURATION = 4
CSV_FILE = "exp1_vosk.tsv"   # TSV for locale safety

# Keyword-based intents
INTENT_KEYWORDS = {
    "living room": "Going to the living room",
    "bedroom": "Going to the bedroom",
    "kitchen": "Going to the kitchen",
    "study room": "Going to the study room"
}

# =========================
# Load Vosk Model
# =========================
print("\nLoading Vosk model...")
model = vosk.Model(MODEL_PATH)
recognizer = vosk.KaldiRecognizer(model, SAMPLE_RATE)
recognizer.SetWords(True)   # <-- IMPORTANT for accuracy

print("Model loaded.")
print("Experiment 1 – Keyword-based Speech Command Execution")
print("Press ENTER to start | Ctrl+C to stop\n")
input("Press ENTER to begin...")

# =========================
# CSV Header
# =========================
header = [
    "no",
    "timestamp",
    "recognized_text",
    "detected_keyword",
    "stt_latency_sec",
    "tts_latency_sec",
    "total_latency_sec",
    "cpu_usage_percent",
    "keyword_accuracy",
    "success_flag"
]

if not os.path.isfile(CSV_FILE):
    with open(CSV_FILE, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f, delimiter="\t")
        writer.writerow(header)

# =========================
# Experiment Loop
# =========================
iteration = 1

try:
    while True:
        print("\n===================================")
        print(f"TEST {iteration} – Speak now")
        print("===================================")

        start_total = time.time()
        cpu_before = psutil.cpu_percent(interval=None)

        # -------- Microphone Input --------
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
        recognized_text = result.get("text", "").lower()
        words = result.get("words", [])
        stt_latency = time.time() - start_stt

        print(f"Recognized: {recognized_text}")

        # -------- Keyword Detection --------
        detected_keyword = None
        for keyword in INTENT_KEYWORDS:
            if keyword in recognized_text:
                detected_keyword = keyword
                break

        if detected_keyword:
            response_text = INTENT_KEYWORDS[detected_keyword]
            success_flag = 1
            keyword_accuracy = 1.0
        else:
            response_text = "Command not recognized. Please repeat."
            success_flag = 0
            keyword_accuracy = 0.0

        # -------- Execution Feedback (TTS) --------
        start_tts = time.time()
        gTTS(response_text, lang="en").save("response.mp3")
        os.system("mpg123 response.mp3")
        tts_latency = time.time() - start_tts

        cpu_after = psutil.cpu_percent(interval=None)
        cpu_usage = (cpu_before + cpu_after) / 2
        total_latency = time.time() - start_total

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # -------- Save Results --------
        row = [
            iteration,
            timestamp,
            recognized_text,
            detected_keyword if detected_keyword else "none",
            round(stt_latency, 3),
            round(tts_latency, 3),
            round(total_latency, 3),
            round(cpu_usage, 2),
            keyword_accuracy,
            success_flag
        ]

        with open(CSV_FILE, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f, delimiter="\t")
            writer.writerow(row)

        print(f"Detected keyword: {detected_keyword}")
        print(f"Accuracy: {keyword_accuracy}")
        print(f"Response: {response_text}")

        iteration += 1

except KeyboardInterrupt:
    print("\nExperiment stopped safely.")
