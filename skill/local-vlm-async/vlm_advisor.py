# -*- coding: utf-8 -*-
"""Asynchronous small-VLM advisor for a real-time control loop.

Drop-in pattern:
    adv = VlmAdvisor()          # loads synchronously
    adv.start()                 # starts the worker
    adv.request(frame, blocks)  # non-blocking, throttled
    colour, reason = adv.latest()

The model is treated as untrusted: answers are validated against the candidate
set supplied with the request.
"""
import os
import threading
import time

os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")

MODEL_ID = os.environ.get("VLM_MODEL", "HuggingFaceTB/SmolVLM-256M-Instruct")

PROMPT = (
    "A robot arm sorts colored blocks from a conveyor belt into boxes. "
    "Detected blocks this frame: {colors}. "
    "Which color should be picked first? "
    "Answer exactly two lines:\n"
    "FIRST: <one of {colors}>\n"
    "REASON: <under 12 words>"
)


class VlmAdvisor(object):
    def __init__(self, enabled=True, min_interval=6.0):
        self.enabled = enabled
        self.min_interval = min_interval
        self._model = None
        self._proc = None
        self._lock = threading.Lock()
        self._job = None
        self._last_req = 0.0
        self._result = None
        self._busy = False
        self._stop = False
        self._lat = 0.0

    # ---------- lifecycle ----------
    def load(self):
        if self._model is not None or not self.enabled:
            return self._model is not None
        try:
            import torch
            from transformers import AutoProcessor, AutoModelForVision2Seq
            torch.set_num_threads(max(2, (os.cpu_count() or 4) // 2))
            self._proc = AutoProcessor.from_pretrained(MODEL_ID)
            self._model = AutoModelForVision2Seq.from_pretrained(
                MODEL_ID, torch_dtype=torch.float32).eval()
            return True
        except Exception as exc:
            print("[vlm] load failed:", exc)
            self.enabled = False
            return False

    def start(self):
        if not self.enabled:
            return
        if not self.load():          # load in the MAIN thread, not the worker
            return
        threading.Thread(target=self._worker, daemon=True).start()

    def stop(self):
        self._stop = True

    # ---------- worker ----------
    def _worker(self):
        if self._model is None:
            return
        while not self._stop:
            job = None
            with self._lock:
                if self._job is not None:
                    job = self._job
                    self._job = None
                    self._busy = True
            if job is None:
                time.sleep(0.2)
                continue
            try:
                answer, reason = self._infer(*job)
                with self._lock:
                    self._result = (answer, reason, time.time())
            except Exception as exc:
                print("[vlm] inference failed:", exc)
            finally:
                with self._lock:
                    self._busy = False

    def _infer(self, frame, blocks):
        import torch
        from PIL import Image

        img = Image.fromarray(frame[:, :, ::-1])
        w, h = img.size
        img = img.resize((448, int(h * 448.0 / max(w, 1))))

        colors = sorted({b[0] for b in blocks}) or ["none"]
        text = PROMPT.format(colors=", ".join(colors))

        msgs = [{"role": "user", "content": [
            {"type": "image"}, {"type": "text", "text": text}]}]
        prompt = self._proc.apply_chat_template(msgs, add_generation_prompt=True)
        inp = self._proc(text=prompt, images=[img], return_tensors="pt")

        t0 = time.time()
        with torch.no_grad():
            out = self._model.generate(**inp, max_new_tokens=40, do_sample=False)
        self._lat = time.time() - t0

        txt = self._proc.batch_decode(
            out[:, inp["input_ids"].shape[1]:], skip_special_tokens=True)[0]

        answer, reason = "", txt.strip().replace("\n", " ")
        for line in txt.split("\n"):
            s = line.strip()
            if s.upper().startswith("FIRST:"):
                answer = s.split(":", 1)[1].strip().lower().rstrip(".")
            elif s.upper().startswith("REASON:"):
                reason = s.split(":", 1)[1].strip()

        if answer not in colors:        # untrusted output -> reject
            answer = ""
        return answer, reason[:60]

    # ---------- public API ----------
    def request(self, frame, blocks):
        if not self.enabled:
            return False
        now = time.time()
        with self._lock:
            if self._busy or self._job is not None:
                return False
            if now - self._last_req < self.min_interval:
                return False
            self._job = (frame.copy(), list(blocks))
            self._last_req = now
        return True

    def latest(self, max_age=40.0):
        with self._lock:
            if self._result is None:
                return None, None
            answer, reason, ts = self._result
            if time.time() - ts > max_age:
                return None, None
        return answer, reason

    def stats(self):
        with self._lock:
            return {"busy": self._busy, "latency": self._lat,
                    "age": (time.time() - self._result[2]) if self._result else -1}
