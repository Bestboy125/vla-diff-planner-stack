# YOLO-World chair fine-tuning

1. Double-click `start_labelimg.bat` and verify every chair box. Use YOLO format and class `chair`.
2. Save every image, including images with no target (an empty label file is valid).
3. Double-click `run_training.bat` after annotation is complete.

Best weights are written to `runs/chair_yolo_world/weights/best.pt`.

Run `python predict.py C:\path\to\image_or_folder` to detect chairs with the trained model.
