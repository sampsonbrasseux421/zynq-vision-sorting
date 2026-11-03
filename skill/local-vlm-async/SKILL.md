---
name: local-vlm-async
description: Attach a small vision-language model as a slow advisor inside a real-time control loop on a CPU-only host, without adding latency to the loop.
license: MIT
---

# Local small VLM as an asynchronous advisor

## 1. When to use

- The host has **no GPU** but you want a vision-language model in the loop.
- The model's output is used for **task-level decisions** (which object first,
  what to do next), not for per-frame control.
- A decision latency of several seconds is acceptable, but the control loop must
  not stall.

## 2. When NOT to use

- The output must drive every frame (use a small detector instead).
- You have a GPU — a larger model may fit, but the async pattern still applies.
- The task can be solved by deterministic rules; adding a model only adds risk.

## 3. What you get

| File | Purpose |
|---|---|
| `vlm_advisor.py` | Self-contained advisor: background worker, request throttling, result cache, graceful degradation |
| `SKILL.md` | Integration pattern, model sizing, failure conditions |

## 4. Measured sizing on CPU

| Model | Params | Load | Inference (CPU, 24 threads) |
|---|---|---|---|
| SmolVLM-256M-Instruct | 256 M | ~6 s | ~10–12 s |
| 7 B class | 7 B | minutes | tens of seconds — unusable |

Image size (256 / 384 / 512 px) barely changes the inference time; the vision
encoder and token generation dominate. **Do not spend time on downscaling.**

## 5. Integration pattern

```
main loop (real-time)                 worker thread (slow)
  |                                     |
  |-- request(frame, detections) ------>|  (throttled, non-blocking)
  |                                     |  preprocess
  |                                     |  generate
  |<-- latest() -> (answer, reason) ----|  cache result + timestamp
  |                                     |
  use answer to bias the NEXT decision
```

Four rules:

1. **Never call the model inline.** A 10 s call inside a 1 s loop destroys the
   loop.
2. **Throttle requests.** One request per N seconds (we use 6 s) and only when
   the worker is idle.
3. **Constrain the answer set.** Put the detected candidates into the prompt and
   validate the answer against them; discard anything else.
4. **Degrade silently.** If the model is missing, slow, or returns garbage, the
   existing rule-based logic must continue unchanged.

## 6. Prompt shape that works for tiny models

Small models cannot read colours reliably from an image alone. Give them the
detections as text and force a two-line answer:

```
A robot arm sorts colored blocks from a conveyor belt into boxes.
Detected blocks this frame: black, yellow, green.
Which color should be picked first?
Answer exactly two lines:
FIRST: <one of black, yellow, green>
REASON: <under 12 words>
```

Then parse strictly:

```python
if answer not in candidates:      # reject
    return None
```

## 7. Bitter lesson

- **Do not load the model inside the worker thread.** Importing heavy libraries
  off the main thread disturbed the GUI event loop on Windows and made the host
  application exit on its own. Load synchronously at startup, then start the
  worker.
- **A 256 M model will occasionally answer nonsense.** Treat every answer as
  untrusted input: validate against the candidate set, and never let it move
  hardware that the rule-based path would not move.

## 8. Failure conditions

- The host is a real-time OS with no background scheduling headroom.
- The decision must be reproducible for a report — a stochastic model may give
  different answers on different runs.
- The candidate set is large or open-ended (then the constrained-prompt trick
  does not apply).
