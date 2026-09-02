import argparse
from pathlib import Path
import shutil

from ultralytics import YOLOWorld


ROOT = Path(__file__).resolve().parent
IMAGES = ROOT / "dataset" / "images" / "all"
LABELS = ROOT / "dataset" / "labels" / "all"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Create initial YOLO chair annotations.")
    parser.add_argument("source", type=Path, help="Directory containing source images")
    return parser.parse_args()


def main() -> None:
    source = parse_args().source.expanduser().resolve()
    IMAGES.mkdir(parents=True, exist_ok=True)
    LABELS.mkdir(parents=True, exist_ok=True)
    image_paths = sorted(p for p in source.iterdir() if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".webp"})
    if not image_paths:
        raise SystemExit(f"No images found in {source}")
    for src in image_paths:
        shutil.copy2(src, IMAGES / src.name)

    model = YOLOWorld("yolov8s-worldv2.pt")
    model.set_classes(["chair"])
    results = model.predict([str(IMAGES / p.name) for p in image_paths], conf=0.12, iou=0.5, imgsz=960, device=0)
    for result in results:
        label_path = LABELS / f"{Path(result.path).stem}.txt"
        lines = []
        if result.boxes is not None:
            for xywhn in result.boxes.xywhn.cpu().tolist():
                lines.append("0 " + " ".join(f"{v:.6f}" for v in xywhn))
        label_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
    (ROOT / "dataset" / "classes.txt").write_text("chair\n", encoding="utf-8")
    print(f"Prepared {len(image_paths)} images and initial labels in {ROOT / 'dataset'}")


if __name__ == "__main__":
    main()
