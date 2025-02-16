import cv2

def test_cameras():
    for i in range(10):  # Try indices 0-9
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"Camera {i} is working")
                cv2.imshow(f"Camera {i}", frame)
                cv2.waitKey(1000)
                cv2.destroyAllWindows()
            cap.release()
        else:
            print(f"Camera {i} is not available")

test_cameras()