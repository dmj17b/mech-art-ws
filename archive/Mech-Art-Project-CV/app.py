import cv2
import numpy as np

WEIGHTS_PATH = "yolov3.weights"
CONFIG_PATH = "yolov3.cfg"

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
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

# Initialize a list to store the points
room_corner_points = []

# Function to handle mouse clicks
def click_event(event, x, y, flags, param):
    global room_corner_points, frame
    if event == cv2.EVENT_LBUTTONDOWN:
        # Store the point coordinates in the list
        room_corner_points.append((x, y))

def annotate_points(frame, points):
    for idx, (x, y) in enumerate(points):
        # Display the clicked point on the video frame
        cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)

        # Show the point number
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, f"{idx + 1}", (x, y), font, 0.5, (0, 0, 255), 2)

def get_room_corners_from_video():
    global frame, room_corner_points
    room_corner_points = []  # Reset points for each function call
    
    cv2.namedWindow("Video")
    cv2.setMouseCallback("Video", click_event)

    # Instructions for the user
    print("Click on the video in the following order:")
    print("1. Bottom left of the room")
    print("2. Top left of the room")
    print("3. Top right of the room")
    print("4. Bottom right of the room")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Draw annotations on the frame
        annotate_points(frame, room_corner_points)

        cv2.imshow("Video", frame)
        key = cv2.waitKey(1) & 0xFF

        # Break the loop when 'q' is pressed or 4 points have been clicked
        if key == ord('q') or len(room_corner_points) == 4:
            break

    cv2.destroyAllWindows()

    # Return the selected points
    return room_corner_points

def convert_room_corners_to_homography_matrix(room_corners):
    # Define the real world coordinates of the room corners
    real_world_corners = np.array([[-2, -2], [-2, +2], [+2, +2], [+2, -2]], dtype=np.float32)

    # Convert the room corners to numpy array
    room_corners = np.array(room_corners, dtype=np.float32)

    # Find the homography matrix
    homography_matrix, _ = cv2.findHomography(room_corners, real_world_corners)
    if homography_matrix is None:
        print("Failed to find a homography matrix.")

    return homography_matrix

def get_persons_locations_from_video():
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

    persons_locations = []
    for i in indices:
        box = boxes[i]
        x, y, w, h = box
        # Append bottom center of the person
        persons_locations.append((x + w // 2, y + h))
    
    return persons_locations

def convert_person_locations_to_room_locations(person_locations, homography_matrix):
    # Convert the person locations to numpy array
    person_locations = np.array(person_locations, dtype=np.float32).reshape(-1, 1, 2)

    # Apply perspective transform to get the room locations
    room_locations = cv2.perspectiveTransform(person_locations, homography_matrix)

    return room_locations

def display_room_locations_heat_map(room_locations):
    
    # Check if room_locations is None
    if room_locations is None:
        print("No room locations to display.")
        room_locations = []

    # Create an empty image to display the heat map
    heat_map = np.zeros((480, 640), dtype=np.uint8)

    room_width = 16
    room_height = 20

    # Draw circles at the room locations
    for room_location in room_locations:
        x, y = room_location[0]
        x = x / room_width + 0.5
        y = -y / room_height + 0.5
        cv2.circle(heat_map, (int(x * 640), int(y * 480)), 5, 255, -1)

    # Display the heat map
    cv2.imshow("Heat Map", heat_map)

if __name__ == "__main__":

    room_corner_points = get_room_corners_from_video()

    for point in room_corner_points:
        print(point)

    homography_matrix = convert_room_corners_to_homography_matrix(room_corner_points)

    print("Homography matrix:")
    print(homography_matrix)

    if homography_matrix is not None:
        while True:
            persons_locations = get_persons_locations_from_video()
            room_locations = convert_person_locations_to_room_locations(persons_locations, homography_matrix)
            display_room_locations_heat_map(room_locations)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break