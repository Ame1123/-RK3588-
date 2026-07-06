from ultralytics import YOLO

# Load a model
model = YOLO('path/to/best.pt')  # load a custom trained

# Export the model
model.export(format='engine',half=True,simplify=True)
