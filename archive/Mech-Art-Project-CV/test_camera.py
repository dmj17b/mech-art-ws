import cv2

def test_camera_feed():
    # Initialize the video capture object. 0 is the ID of the built-in camera, change if you have multiple cameras.
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open video device.")
        return

    print("Press 'q' to exit")

    # Continuously capture frames from the camera and display them
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # If frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        # Display the resulting frame
        cv2.imshow('Camera Feed', frame)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    test_camera_feed()
