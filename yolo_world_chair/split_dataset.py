from pathlib import Path
import random
import shutil


ROOT = Path(__file__).resolve().parent
DATASET = ROOT / "dataset"
SEED = 42


def main() -> None:
    images = sorted((DATASET / "images" / "all").glob("*"))
    if not images:
        raise SystemExit("No prepared images. Run auto_annotate.py first.")
    missing = [p.name for p in images if not (DATASET / "labels" / "all" / f"{p.stem}.txt").exists()]
    if missing:
        raise SystemExit(f"Missing label files: {missing}")
    random.Random(SEED).shuffle(images)
    val_count = max(2, round(len(images) * 0.2))
    val_names = {p.name for p in images[:val_count]}
    for split in ("train", "val"):
        for kind in ("images", "labels"):
            target = DATASET / kind / split
            target.mkdir(parents=True, exist_ok=True)
            for old in target.iterdir():
                if old.is_file():
                    old.unlink()
    for image in images:
        split = "val" if image.name in val_names else "train"
        shutil.copy2(image, DATASET / "images" / split / image.name)
        label = DATASET / "labels" / "all" / f"{image.stem}.txt"
        shutil.copy2(label, DATASET / "labels" / split / label.name)
    print(f"Split {len(images) - val_count} train / {val_count} val images")


if __name__ == "__main__":
    main()
