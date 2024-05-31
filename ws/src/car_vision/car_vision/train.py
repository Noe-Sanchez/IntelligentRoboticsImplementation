from ultralytics import YOLO
import pathlib
import os
 
BASE_PATH = pathlib.Path(__file__).parent.absolute()
dataset_path = os.path.join(BASE_PATH, 'dataset_final')

# Load the model.
model = YOLO(f'{BASE_PATH}/yolov8n.pt')
 
# Training.
results = model.train(
   data=dataset_path + '/data.yaml',
   imgsz=640,
   epochs=40,
   batch=8,
   name='/yolov8n_custom'
)