import argparse
from pathlib import Path

from ultralytics import YOLOWorld


ROOT = Path(__file__).resolve().parent
MODEL = ROOT / "runs" / "chair_yolo_world" / "weights" / "best.pt"
def main() -> None:
    parser = argparse.ArgumentParser(description="Detect the fine-tuned chair class.")
    parser.add_argument("source", nargs="?", default=str(ROOT / "dataset" / "images" / "all"))
    parser.add_argument("--conf", type=float, default=0.10, help="Confidence threshold")
    args = parser.parse_args()
    model = YOLOWorld(str(MODEL))
    results = model.predict(
        source=args.source, conf=args.conf, imgsz=960, device=0, save=True,
        project=str(ROOT / "runs"), name="chair_predictions", exist_ok=True,
    )
    total = sum(len(result.boxes) for result in results)
    print(f"images={len(results)} detections={total}")
    print(f"visualizations={ROOT / 'runs' / 'chair_predictions'}")


if __name__ == "__main__":
    main()
