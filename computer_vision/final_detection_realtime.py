import cv2
import numpy as np
import glob
import os
import time

def get_images_from_folder(folder_path):
    """Get all images from a folder and return their paths."""
    image_paths = glob.glob(os.path.join(folder_path, '*.jpg'))
    all_images = []
    for img_path in image_paths:
        image = cv2.imread(img_path)
        if image is None:
            print(f"Failed to load {img_path}")
            continue
        all_images.append((img_path, image))
    return all_images
        

def detect_cube_2d(image, lower_color_bound, upper_color_bound):

    # Convert to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Optional: Smooth a little to reduce noise
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

    # Threshold green
    mask = cv2.inRange(hsv, lower_color_bound, upper_color_bound)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)


        return (x, y, w, h)

    else:
        print(f"No cube detected.")
        return None

def draw_cube(image, cube_info):
    """Draw the detected cube on the image."""
    if cube_info is not None:
        (x, y, w, h) = cube_info
        center_x = x + w // 2
        center_y = y + h // 2
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(image, (center_x, center_y), 5, (0, 0, 255), -1)
        cv2.imshow('Detected Cube', image)
        cv2.waitKey(100)
    return image

def get_world_frame_coordinates(cube_info):
    if cube_info is not None:
        x, y, w, h = cube_info
        center_x = x + w // 2
        center_y = y + h // 2

        # Unproject the 2D center into 3D space
        z = fixed_depth
        X = (center_x - cx) * z / fx
        Y = (center_y - cy) * z / fy

        # print(f"3D coordinates of the cube center: X={X:.3f}, Y={Y:.3f}, Z={z:.3f}")

        # transform the coordinates to the camera frame
        camera_frame = np.array([X, Y, z])
        # print(f"Camera frame coordinates: {camera_frame}")
        
        # transform the coordinates to the world frame
        # Assuming a simple rotation and translation for demonstration
        # In a real scenario, you would use the actual rotation and translation matrices
        rotation_matrix = np.array([[0, -1, 0],
                                    [-1, 0, 0],
                                    [0, 0, -1]])
        translation_vector = np.array([0.21, 0, fixed_depth])
        world_frame = rotation_matrix @ camera_frame + translation_vector
        
        return world_frame
    
    return None

def main(detect_color='red'):
    
    if detect_color == 'red':
        lower_color_bound = np.array([[0, 100, 50]]) # red
        upper_color_bound = np.array([[10, 255, 255]]) # red
    elif detect_color == 'blue':
        lower_color_bound = np.array([100, 100, 50]) # blue 
        upper_color_bound = np.array([130, 255, 255]) # blue
    else:
        raise ValueError("Invalid color specified. Use 'red' or 'blue'.")

    # Open a connection to the first camera (index 0)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    print("Press 'q' to quit.")

    while True:
        # Capture frame-by-frame
        ret, image = cap.read()
        # time.sleep(0.1)  # Optional: Add a small delay to avoid high CPU usage

        if not ret:
            print("Error: Failed to capture image.")
            break
        cube_info = detect_cube_2d(image, lower_color_bound, upper_color_bound)
        
        if vis:
            image_with_cube = draw_cube(image.copy(), cube_info)
            
        world_coords = get_world_frame_coordinates(cube_info)
        
        if world_coords is not None:
            print(f"World frame coordinates: {world_coords}")
        
        # Press 'q' to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        

            
if __name__ == "__main__":
    
    detected_color = 'blue'  # Change to 'blue' if needed
    
    vis = True  # Set to True to visualize the camera feed and detected cube
    
    # Camera intrinsics (modify with your real calibration results)
    fx = 347.79486758  # example value
    fy = 347.99189723  # example value
    cx = 330.71293139  # example value
    cy = 265.09413408  # example value

    camera_matrix = np.array([[fx, 0, cx],
                            [0, fy, cy],
                            [0,  0,  1]])
    dist_coeffs =np.array([-0.34131066,  0.15251024, -0.00086168,  0.00074253, -0.03798206])  # example value

    fixed_depth = 0.215 # meters
    
    main(detected_color)
    cv2.destroyAllWindows()