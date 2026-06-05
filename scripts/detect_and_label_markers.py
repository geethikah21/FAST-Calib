import cv2
import numpy as np

def detect_aruco_markers():
    # Read image
    # fname = '00000890.png'
    # fname = '00000619.png'
    # fname = '00000994.png'
    # fname = '00000872.png'
    # fname = '00000979.png'
    # fname = '00000982.png'
    # fname = '00000982_cropped.png'
    # fname = '00000010_20260220_p1.png'
    # fname = '00000340_20260220_p2.png'
    # fname = '00000120_20260220_p3.png'
    # fname = '00000200_20260312_indoor1_p3.png'
    fname = '00000200_20260312_indoor2_p4.png'
    img = cv2.imread(fname)

    # print(hasattr(cv2, "aruco"))  # should be True

    # Run ArUco detection on enhanced image
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    detector = cv2.aruco.ArucoDetector(aruco_dict)

    corners, ids, rejected = detector.detectMarkers(img)

    # print(corners)
    # print(ids)

    cv2.aruco.drawDetectedMarkers(img, corners, ids)
    cv2.imshow("Detected markers", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return corners, ids

def select_aruco_corners():
    # Should select corners for markers in cw order starting with top left
    #   tl, tr, br, bl
    # Order that the markers are in doesn't matter as long as the ids correspond
    # to the marker order correctly

    # Global list to store selected points
    selected_points = []

    # Mouse callback function
    def click_event(event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Store the coordinates
            selected_points.append((x, y))
            # Print coordinates to console for verification
            print(f"Point selected at: ({x}, {y})")

            # Draw a small circle to mark the point on the image
            # Use radius=5 and a negative thickness for a filled circle
            cv2.circle(img, (x, y), 5, (0, 255, 0), -1)
            cv2.imshow('image', img) # Update the display

    # --- Main execution ---

    # 1. Load an image (replace 'path_to_your_image.jpg' with your image file path)
    # fname = '00000890.png'
    # fname = '00000619.png'
    # fname = '00000994.png'
    # fname = '00000872.png'
    # fname = '00000979.png'
    # fname = '00000982.png'
    # fname = '00000982_cropped.png'
    # fname = '00000010_20260220_p1.png'
    # fname = '00000340_20260220_p2.png'
    # fname = '00000120_20260220_p3.png'
    # fname = '00000200_20260312_indoor1_p3.png'
    fname = '00000200_20260312_indoor2_p4.png'
    img = cv2.imread(fname)
    if img is None:
        print("Error: Could not load image. Check the file path.")
        exit()

    # 2. Create a window and set the mouse callback
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', click_event)

    print("Click on the image window to select points. Press 'esc' to exit.")

    # 3. Display the image and wait for keypress
    while True:
        cv2.imshow('image', img)
        # Wait for a key press (1ms delay), break the loop if 'esc' (27) is pressed
        k = cv2.waitKey(1) & 0xFF
        if k == 32: # ASCII for spacebar
            break

    # 4. Cleanup
    cv2.destroyAllWindows()

    # group the points
    grouped_selections = []
    curr_tag_points = []
    for i, pt in enumerate(selected_points):
        if i % 4 == 0 and i != 0:
            grouped_selections.append(np.vstack(curr_tag_points, dtype=np.float32))
            curr_tag_points = [pt]
        else:
            curr_tag_points.append(pt)
    # handle final group
    grouped_selections.append(np.vstack(curr_tag_points, dtype=np.float32))
    grouped_selections = tuple(grouped_selections)

    # ids = np.array([[1], [2], [4], [3]])
    ids = np.array([[4], [3]])

    # Print all collected points at the end
    # print("\nAll selected points:", grouped_selections)
    # print(f"Ids: {ids}")

    return grouped_selections, ids

def compute_error(auto_corners, auto_ids, manual_corners, manual_ids):
    auto_dict = {int(i): c for i, c in zip(auto_ids.flatten(), auto_corners)}
    manual_dict = {int(i): c for i, c in zip(manual_ids.flatten(), manual_corners)}

    print(f"Auto dict: {auto_dict}")
    print(f"Manual dict: {manual_dict}")

    for i in range(1, 5):
        if i in auto_dict and i in manual_dict:
            print(f"Err for marker {i}")
            print(auto_dict[i] - manual_dict[i])

def format_corners(corners):
    '''
    format from python into c++ equivalent version of corners list
    '''

    for marker in corners:
        print("{")
        for x, y in marker:
            print(f"    cv::Point2f({int(x)}, {int(y)}),")
        print("},")

if __name__ == '__main__':
    auto_corners, auto_ids = detect_aruco_markers()
    manual_corners, manual_ids = select_aruco_corners()

    compute_error(auto_corners, auto_ids, manual_corners, manual_ids)

    format_corners(manual_corners)