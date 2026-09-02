from pathlib import Path

from ultralytics import YOLOWorld


ROOT = Path(__file__).resolve().parent


def main() -> None:
    model = YOLOWorld("yolov8s-worldv2.pt")
    model.train(
        data=str(ROOT / "chair.yaml"), epochs=100, imgsz=960, batch=8,
        device=0, workers=4, patience=25, cache=True,
        project=str(ROOT / "runs"), name="chair_yolo_world", exist_ok=True,
    )


if __name__ == "__main__":
    main()
