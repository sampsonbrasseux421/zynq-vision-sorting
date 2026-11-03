# -*- coding: utf-8 -*-
"""
vlm_advisor.py —— 本地视觉语言模型（SmolVLM-256M）任务级决策

作用：
    看当前画面 + 检测到的色块列表，给出"先抓哪个、为什么"的建议。
    因为 CPU 推理约 10 秒，所以放在后台线程异步运行，
    结果只作为"下一次抓取"的偏好，不阻塞实时控制。

用法：
    adv = VlmAdvisor()
    adv.start()
    adv.request(frame, blocks)          # blocks = [(color, px, py), ...]
    order, reason = adv.latest()        # 取最近一次结果（可能为 None）
"""
import os
import sys
import threading
import time

os.environ.setdefault("HF_HUB_DISABLE_SYMLINKS_WARNING", "1")
os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")

MODEL_ID = "HuggingFaceTB/SmolVLM-256M-Instruct"

PROMPT_TMPL = (
    "A robot arm sorts colored blocks from a conveyor belt into boxes. "
    "Detected blocks this frame: {colors}. "
    "Blocks closer to the left side of the belt will leave the workspace sooner. "
    "Which color should be picked first? "
    "Answer exactly two lines:\n"
    "FIRST: <one of {colors}>\n"
    "REASON: <under 12 words>"
)


class VlmAdvisor(object):
    def __init__(self, enabled=True):
        self.enabled = enabled
        self._model = None
        self._proc = None
        self._lock = threading.Lock()
        self._job = None          # (frame copy, blocks)
        self._last_req = 0.0
        self._result = None       # (order_color, reason, timestamp)
        self._busy = False
        self._thread = None
        self._stop = False
        self._lat = 0.0
        self._calls = 0

    # ---------- 加载 ----------
    def load(self):
        if self._model is not None or not self.enabled:
            return self._model is not None
        try:
            import torch
            from transformers import AutoProcessor, AutoModelForVision2Seq
            torch.set_num_threads(max(2, (os.cpu_count() or 4) // 2))
            print("[VLM] 加载 %s ..." % MODEL_ID, flush=True)
            t0 = time.time()
            self._proc = AutoProcessor.from_pretrained(MODEL_ID)
            self._model = AutoModelForVision2Seq.from_pretrained(
                MODEL_ID, torch_dtype=torch.float32).eval()
            print("[VLM] 就绪，加载 %.1fs" % (time.time() - t0), flush=True)
            return True
        except Exception as e:
            print("[VLM] 加载失败:", e, flush=True)
            self.enabled = False
            return False

    # ---------- 后台线程 ----------
    def start(self):
        if not self.enabled:
            return
        # 同步加载：在线程里 import transformers/PIL 会抢占 Windows 消息循环，
        # 导致主程序的 cv2.waitKey 返回异常值（表现为程序自己退出）
        if not self.load():
            return
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

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
            frame, blocks = job
            try:
                color, reason = self._infer(frame, blocks)
                with self._lock:
                    self._result = (color, reason, time.time())
                print("[VLM] 建议先抓 %s —— %s" % (color, reason), flush=True)
            except Exception as e:
                print("[VLM] 推理失败:", e, flush=True)
            finally:
                with self._lock:
                    self._busy = False

    # ---------- 推理 ----------
    def _infer(self, frame, blocks):
        import torch
        from PIL import Image

        img = Image.fromarray(frame[:, :, ::-1])       # BGR -> RGB
        w, h = img.size
        scale = 448.0 / max(w, 1)
        img = img.resize((448, int(h * scale)))

        colors = sorted({b[0] for b in blocks}) or ["none"]
        prompt_txt = PROMPT_TMPL.format(colors=", ".join(colors))

        msgs = [{"role": "user", "content": [
            {"type": "image"},
            {"type": "text", "text": prompt_txt},
        ]}]
        prompt = self._proc.apply_chat_template(msgs, add_generation_prompt=True)
        inp = self._proc(text=prompt, images=[img], return_tensors="pt")
        t0 = time.time()
        with torch.no_grad():
            out = self._model.generate(**inp, max_new_tokens=40,
                                       do_sample=False)
        txt = self._proc.batch_decode(
            out[:, inp["input_ids"].shape[1]:], skip_special_tokens=True)[0]
        self._lat = time.time() - t0
        self._calls += 1

        color, reason = "", txt.strip().replace("\n", " ")
        for line in txt.split("\n"):
            s = line.strip()
            if s.upper().startswith("FIRST:"):
                color = s.split(":", 1)[1].strip().lower().rstrip(".")
            elif s.upper().startswith("REASON:"):
                reason = s.split(":", 1)[1].strip()
        if color not in colors:
            # 兜底：只在候选颜色里找
            color = ""
            low = txt.lower()
            for c in colors:
                if c in low:
                    color = c
                    break
        return color, reason[:60]

    # ---------- 对外接口 ----------
    def request(self, frame, blocks, min_interval=6.0):
        """提交一次决策请求（限频）。"""
        if not self.enabled:
            return False
        now = time.time()
        with self._lock:
            if self._busy or self._job is not None:
                return False
            if now - self._last_req < min_interval:
                return False
            self._job = (frame.copy(), list(blocks))
            self._last_req = now
        return True

    def latest(self, max_age=40.0):
        with self._lock:
            if self._result is None:
                return None, None
            color, reason, ts = self._result
            if time.time() - ts > max_age:
                return None, None
        return color, reason

    def stats(self):
        with self._lock:
            return {"busy": self._busy, "calls": self._calls,
                    "lat": self._lat,
                    "age": (time.time() - self._result[2]) if self._result else -1}

    def stop(self):
        self._stop = True


if __name__ == "__main__":
    import cv2
    import numpy as np
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    adv = VlmAdvisor()
    adv.start()

    img = np.full((600, 1024, 3), 210, np.uint8)
    cv2.rectangle(img, (150, 300), (250, 400), (0, 0, 0), -1)
    cv2.rectangle(img, (420, 340), (520, 440), (0, 200, 255), -1)
    cv2.rectangle(img, (700, 260), (800, 360), (0, 160, 0), -1)

    print("提交决策请求...")
    adv.request(img, [("black", 200, 350), ("yellow", 470, 390), ("green", 750, 310)])
    for _ in range(120):
        c, r = adv.latest()
        if c:
            print("\n结果: 先抓 %s\n理由: %s" % (c, r))
            break
        time.sleep(0.5)
    else:
        print("超时未返回")
    print("统计:", adv.stats())
    adv.stop()
