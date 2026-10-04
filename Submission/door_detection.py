from ultralytics import YOLO


model2 = YOLO("trained_models/best_yolo_26n_inside_door_clean.pt")
res = model2.predict(["../Images/door_sums_lounge.jpg", "../Images/door_kitchen.jpg", "../Images/entrance_door.jpg"], save=False, conf=0.1)

for item in res:
    print(item.boxes)