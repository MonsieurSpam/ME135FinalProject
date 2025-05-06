import cv2
import numpy as np

# ==== CONFIGURATION ====
CONFIG = {
    'fx': 347.79486758,
    'fy': 347.99189723,
    'cx': 330.71293139,
    'cy': 265.09413408,
    'dist_coeffs': np.array([-0.34131066, 0.15251024, -0.00086168, 0.00074253, -0.03798206]),
    'fixed_depth': 0.215,  # in meters
    'rotation_matrix': np.array([
        [0, -1, 0],
        [-1, 0, 0],
        [0, 0, -1]
    ]),
    'translation_vector': np.array([0.21, 0, 0.215]),  # Update with actual values if needed
    'visualize': True,
    'detected_color': 'red',  # 'red' or 'blue'
}

COLOR_RANGES = {
    'red': (np.array([0, 100, 50]), np.array([10, 255, 255])),
    'blue': (np.array([100, 100, 50]), np.array([130, 255, 255]))
}


# ==== FUNCTIONS ====
def detect_cube_2d(image, lower_bound, upper_bound):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    return cv2.boundingRect(largest)


def draw_cube(image, bbox):
    if bbox:
        x, y, w, h = bbox
        center = (x + w // 2, y + h // 2)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(image, center, 5, (0, 0, 255), -1)
        cv2.imshow('Detected Cube', image)


def compute_world_coords(bbox, config):
    if bbox is None:
        return None

    x, y, w, h = bbox
    cx_img = x + w // 2
    cy_img = y + h // 2
    z = config['fixed_depth']

    X = (cx_img - config['cx']) * z / config['fx']
    Y = (cy_img - config['cy']) * z / config['fy']
    camera_coords = np.array([X, Y, z])

    world_coords = config['rotation_matrix'] @ camera_coords + config['translation_vector']
    return world_coords


def main(config):
    color = config['detected_color']
    if color not in COLOR_RANGES:
        raise ValueError("Color must be 'red' or 'blue'")
    lower, upper = COLOR_RANGES[color]

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera.")

    print("Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture image.")
            break

        bbox = detect_cube_2d(frame, lower, upper)

        if config['visualize']:
            draw_cube(frame.copy(), bbox)
            cv2.waitKey(100)

        coords = compute_world_coords(bbox, config)
        if coords is not None:
            print(f"World Coordinates: {coords}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


# ==== RUN ====
if __name__ == "__main__":
    main(CONFIG)
