import cv2
import numpy as np

WEIGHTS_PATH = "yolov3-tiny.weights"
CONFIG_PATH = "yolov3-tiny.cfg"

# Load YOLO
net = cv2.dnn.readNet(WEIGHTS_PATH, CONFIG_PATH)
layer_names = net.getLayerNames()
output_layer_indices = net.getUnconnectedOutLayers().flatten()
output_layers = [layer_names[i - 1] for i in output_layer_indices]

# Loading labels
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
person_class_id = classes.index("person")  # Get the index for 'person'

# Initialize video
cap = cv2.VideoCapture(0)  # Use 0 for webcam

while True:
    _, frame = cap.read()
    
    # Convert frame to blob
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)
    
    # Information to display
    class_ids, confidences, boxes = [], [], []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if class_id == person_class_id and confidence > 0.5:
                # Person detected
                center_x, center_y, width, height = (detection[0:4] * np.array([frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])).astype('int')
                x = int(center_x - width / 2)
                y = int(center_y - height / 2)
                boxes.append([x, y, width, height])
                confidences.append(float(confidence))
                class_ids.append(class_id)
    
    # Non-max suppression to avoid multiple boxes
    indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    
    if len(indices) > 0 and isinstance(indices, tuple):
        indices = indices[0]

    for i in indices:
        box = boxes[i]
        x, y, w, h = box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    cv2.imshow("Image", frame)
    key = cv2.waitKey(1)
    if key == 27:  # ESC key to break
        break

cap.release()
cv2.destroyAllWindows()
