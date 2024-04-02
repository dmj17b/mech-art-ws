import cv2

# Initialize a list to store the points
points = []

# Function to handle mouse clicks
def click_event(event, x, y, flags, param):
    global points, frame
    if event == cv2.EVENT_LBUTTONDOWN:
        # Store the point coordinates in the list
        points.append((x, y))

        # Since we draw on the frame in the main loop now, no need to draw here

def annotate_points(frame, points):
    for idx, (x, y) in enumerate(points):
        # Display the clicked point on the video frame
        cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)

        # Show the point number
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, f"{idx + 1}", (x, y), font, 0.5, (0, 0, 255), 2)

def get_room_corners_from_video(video_source=0):
    global frame, points
    points = []  # Reset points for each function call

    cap = cv2.VideoCapture(video_source)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    cv2.namedWindow("Video")
    cv2.setMouseCallback("Video", click_event)

    # Instructions for the user
    print("Click on the video in the following order:")
    print("1. Bottom left of the room")
    print("2. Top left of the room")
    print("3. Top right of the room")
    print("4. Bottom right of the room")
    print("Press 'q' to quit after selecting all points.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Draw annotations on the frame
        annotate_points(frame, points)

        cv2.imshow("Video", frame)
        key = cv2.waitKey(1) & 0xFF

        # Break the loop when 'q' is pressed or 4 points have been clicked
        if key == ord('q') or len(points) == 4:
            break

    cap.release()
    cv2.destroyAllWindows()

    # Return the selected points
    return points

# Example usage
# points = get_room_corners_from_video()
# print("Selected points:", points)

if __name__ == '__main__':
    points = get_room_corners_from_video()
    print("Selected points:", points)
    print("Bottom left:", points[0])
    print("Top left:", points[1])
    print("Top right:", points[2])
    print("Bottom right:", points[3])
