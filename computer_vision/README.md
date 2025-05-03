# Some pre-defined parameters

### size of the cube
2.5 cm - 2.6 cm

### Depth (platform height):
21.5 cm


### Camera coordniate system
- x: right
- y: downward
- z: inward


# Run
```bash
python final_detection_realtime.py
```

- change `cap = cv2.VideoCapture(0)` if it's not using the right camera
- change `detected_color` to choose different colors
- change `visualize` to turn off the real-time visualization of the detected output