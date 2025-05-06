import cv2
import time
import os
def capture_images():
    # Open a connection to the first camera (index 0)
    cap = cv2.VideoCapture(0)
    
    save_path = 'final_images'
    # Create the directory if it doesn't exist
    
    if not os.path.exists(save_path):
        os.makedirs(save_path)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    print("Press 'q' to quit.")

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture image.")
            break
        
        # Check if 's' key is pressed to save the image
        if cv2.waitKey(1) & 0xFF == ord('s'):
            # Generate a filename based on the current timestamp
            filename = f"{save_path}/image_{int(time.time())}.jpg"
            # Save the image
            cv2.imwrite(filename, frame)
            print(f"Image saved as {filename}")

        # Display the resulting frame
        cv2.imshow('Camera Feed', frame)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_images()