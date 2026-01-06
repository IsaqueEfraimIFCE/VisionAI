# ws_yolo_tts_separado_reativo.py
# Servidor WebSocket YOLO + TTS local separado (reativo)
# - WebSocket recebe imagens, executa YOLO e atualiza fila de alertas
# - TTS roda em thread independente e fala o alerta mais comum a cada 2s (sem time.sleep bloqueante)

import asyncio
import websockets
from PIL import Image
from ultralytics import YOLO
import cv2
import numpy as np
import warnings
import os
import time
from collections import Counter, deque
import pyttsx3
from concurrent.futures import ThreadPoolExecutor
import threading

warnings.filterwarnings("ignore")

# ============ Configurações ============ #
os.makedirs("frames_inferidos", exist_ok=True)
executor = ThreadPoolExecutor(max_workers=6)

# ============ Carrega YOLO ============ #
model = YOLO('yolov12s.pt')

# ============ Buffer de alertas compartilhado ============ #
alert_queue = deque(maxlen=32)
alert_lock = threading.Lock()

# --------- Função TTS (fala localmente) --------- #
def speak_tts_local(word: str):
    try:
        engine = pyttsx3.init()
        voice_id = "HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Speech\\Voices\\Tokens\\TTS_MS_PT-BR_MARIA_11.0"
        engine.setProperty('voice', voice_id)
        rate = engine.getProperty('rate')
        engine.setProperty('rate', int(rate * 0.8))
        engine.say(word)
        engine.runAndWait()
        engine.stop()
    except Exception:
        pass

# --------- Thread TTS (reativa, sem sleep bloqueante) --------- #
def tts_loop():
    last_speak_time = 0.0
    interval = 2.0
    last_alert_spoken = None

    while True:
        now = time.monotonic()
        with alert_lock:
            has_alerts = bool(alert_queue)

        if has_alerts and (now - last_speak_time) >= interval:
            with alert_lock:
                counts = Counter(alert_queue)
                most_common_alert, _ = counts.most_common(1)[0]
                alert_queue.clear()

            if most_common_alert != last_alert_spoken:
                speak_tts_local(most_common_alert)
                last_alert_spoken = most_common_alert

            last_speak_time = now

        time.sleep(0.05)

# --------- Mapeamento de alertas --------- #
def alert_to_word(alert_msg: str) -> str:
    mapping = {
        "Mover para direita": "Direita",
        "Mover para esquerda": "Esquerda",
        "Parar": "Parar",
        "Abaixar": "Abaixar",
        "Desviar para direita": "Direita",
        "Desviar para esquerda": "Esquerda",
    }
    return mapping.get(alert_msg, "Alerta")

# --------- Inferência YOLO + Lógica --------- #
def infer_and_get_alerts(frame_rgb):
    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    results = model.predict(frame_bgr, imgsz=320, verbose=False)
    result = results[0]
    boxes = result.boxes
    names = result.names
    height, width, _ = frame_bgr.shape
    alert_msgs = []

    for box in boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cls = int(box.cls[0])
        conf = float(box.conf[0])
        label = f"{names[cls]} ({conf:.2f})"

        box_center_x = (x1 + x2) / 2
        box_center_y = (y1 + y2) / 2
        box_height = y2 - y1
        proximity = box_height / height

        if proximity > 0.5:
            if box_center_x < width * 0.4:
                alert_msgs.append("Direita")
            elif box_center_x > width * 0.6:
                alert_msgs.append("Esquerda")
            else:
                alert_msgs.append("Parar")
        elif box_center_y > height * 0.7 and box_height / height > 0.2:
            alert_msgs.append("Abaixar")
        elif box_center_y < height * 0.25:
            if box_center_x < width * 0.5:
                alert_msgs.append("Direita")
            else:
                alert_msgs.append("Esquerda")

    return frame_bgr, alert_msgs

# --------- WebSocket handler --------- #
async def handler(websocket, path):
    frame_count = 0

    try:
        async for message in websocket:
            if isinstance(message, str):
                continue

            frame_count += 1
            np_arr = np.frombuffer(message, np.uint8)
            frame_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame_bgr is None:
                continue

            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            loop = asyncio.get_running_loop()
            _, alerts = await loop.run_in_executor(executor, infer_and_get_alerts, frame_rgb)

            if alerts:
                with alert_lock:
                    alert_queue.extend(alerts)

            await websocket.send(f'{{"type": "ok", "frame": {frame_count}}}')

    except Exception:
        pass
    finally:
        cv2.destroyAllWindows()

# --------- Inicialização --------- #
async def main():
    threading.Thread(target=tts_loop, daemon=True).start()
    server = await websockets.serve(handler, "0.0.0.0", 8765, max_size=None, max_queue=None)
    await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())

