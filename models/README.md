# =============================================================================
# Clear-Run YOLO Models
# =============================================================================

This directory contains the trained YOLO models for FOD detection.

## Directory Structure

```
models/
├── yolov11/
│   ├── fod_detector.pt      # Main trained model
│   ├── fod_detector.onnx    # ONNX export for TensorRT
│   └── config.yaml          # Training configuration
├── yolov12/
│   └── (future model files)
└── README.md
```

## Downloading Models

The trained models are stored separately due to file size. Download them:

```bash
# Download from your model storage
wget https://your-storage/models/fod_detector.pt -O models/yolov11/fod_detector.pt
```

## Training Your Own Model

### 1. Prepare Dataset

Organize your FOD dataset in YOLO format:
```
dataset/
├── train/
│   ├── images/
│   └── labels/
├── val/
│   ├── images/
│   └── labels/
└── data.yaml
```

### 2. Train Model

```python
from ultralytics import YOLO

# Load base model
model = YOLO('yolo11n.pt')

# Train
results = model.train(
    data='dataset/data.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
    name='fod_detector'
)
```

### 3. Export for Deployment

```python
# Export to ONNX (for TensorRT on Jetson)
model.export(format='onnx', simplify=True)

# Export to TensorRT directly
model.export(format='engine', device=0)
```

## FOD Classes

The model is trained to detect:

| Class ID | Class Name | Description |
|----------|------------|-------------|
| 0 | bolt | Bolts and screws |
| 1 | nut | Nuts and lock nuts |
| 2 | rivet | Pop rivets |
| 3 | washer | Flat and lock washers |
| 4 | screw | Various screws |
| 5 | wire | Wire fragments |
| 6 | fragment | Metal fragments |
| 7 | tool | Hand tools |
| 8 | luggage_tag | Luggage identification tags |
| 9 | rubber | Rubber pieces, tire fragments |
| 10 | metal_piece | Generic metal debris |
| 11 | plastic | Plastic debris |
| 12 | unknown_fod | Unclassified FOD |

## Model Performance

Target metrics for runway deployment:

| Metric | Target | Notes |
|--------|--------|-------|
| mAP@0.5 | > 0.85 | Primary accuracy metric |
| mAP@0.5:0.95 | > 0.60 | Strict accuracy |
| Inference Time | < 50ms | On Jetson Orin Nano |
| Recall | > 0.90 | Minimize missed detections |

## Synthetic Data Augmentation (SRIA)

We use Synthetic Randomized Image Augmentation to address:
- Class imbalance (rare FOD types)
- Limited real-world FOD data
- Varying lighting conditions
- Different runway surfaces

Augmentation techniques:
- Random placement on runway backgrounds
- Lighting variation (day/night/overcast)
- Scale variation (simulating altitude changes)
- Motion blur
- Weather effects (rain, fog)
