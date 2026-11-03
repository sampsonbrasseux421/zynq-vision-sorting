# Models used on the host

## 1. Detector

| Item | Value |
|---|---|
| Architecture | YOLOv8 (Ultralytics) |
| Task | colour-block detection on the conveyor |
| Input | 320 × 320, letterboxed from 1280 × 720 |
| Classes | blue / yellow / red / green / black |
| Weights | `best.pt` (project-trained, not shipped — see note) |
| Backend | PyTorch, CPU |
| Post-process | confidence threshold 0.6, vertical ROI y = 120…690 |

> The trained weights are not committed to this repository. Train with
> `data/train` using Ultralytics, or substitute any detector that returns
> bounding boxes with a class name.

## 2. Task-level advisor

| Item | Value |
|---|---|
| Model | `HuggingFaceTB/SmolVLM-256M-Instruct` |
| Params | 256 M |
| Precision | fp32 on CPU |
| Load time | ~6 s |
| Inference | ~10–12 s per request |
| Role | chooses which colour to pick next; never drives motion directly |

The advisor runs in a background thread with a 6 s request throttle. Its output
is validated against the set of detected colours and discarded if unknown.
See `skill/local-vlm-async/` for the pattern and the failure conditions.
