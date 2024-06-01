import cv2
import jetson.inference
import jetson.utils
import numpy as np
from scipy.optimize import linear_sum_assignment
import pyudev

# Function to calculate IoU
def calculate_iou(boxA, boxB):
    # Determine the coordinates of the intersection rectangle
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    # Compute the area of intersection
    interArea = max(0, xB - xA) * max(0, yB - yA)
    if interArea == 0:
        return 0

    # Compute the area of both rectangles
    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])

    # Compute the intersection over union
    iou = interArea / float(boxAArea + boxBArea - interArea)
    return iou

# Function to perform tracking with Hungarian algorithm
def track_objects(tracks, detections):
    if len(tracks) == 0:
        for i, det in enumerate(detections):
            tracks[i] = det
        return tracks

    # Create cost matrix based on IoU, we want to minimize the negative IoU in the assignment problem
    cost_matrix = np.zeros((len(tracks), len(detections)))
    for t_idx, track in enumerate(tracks.values()):
        for d_idx, detection in enumerate(detections):
            cost_matrix[t_idx, d_idx] = -calculate_iou(track, detection)

    # Solve the assignment problem
    row_ind, col_ind = linear_sum_assignment(cost_matrix)

    new_tracks = {}
    assigned_detections = set()
    for r, c in zip(row_ind, col_ind):
        if cost_matrix[r, c] < -0.3:  # Using -0.3 as an example threshold to consider a match
            new_tracks[r] = detections[c]
            assigned_detections.add(c)

    # Update tracks with new detections
    for i, det in enumerate(detections):
        if i not in assigned_detections:
            new_tracks[len(new_tracks)] = det

    return new_tracks


context = pyudev.Context()
devices = list(context.list_devices(subsystem='video4linux'))

# Webcam name
webcam_name = 'FHD Webcam: FHD Webcam'

# Finding the webcam
webcam_device = next((device for device in devices if device.properties.get('ID_V4L_PRODUCT') == webcam_name), None)

# Initialize the detection network
net = jetson.inference.detectNet("peoplenet", threshold=0.5)

if webcam_device:
    webcam_index = int(webcam_device.sys_name.replace('video', ''))
    cap = cv2.VideoCapture(webcam_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

    tracked_objects = {}
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Convert frame to CUDA image
        cuda_frame = jetson.utils.cudaFromNumpy(frame)

        # Detect objects in the frame
        detections = net.Detect(cuda_frame)
        detected_boxes = [[det.Left, det.Top, det.Right, det.Bottom] for det in detections if det.ClassID == 0]

        # Track objects using Hungarian algorithm
        tracked_objects = track_objects(tracked_objects, detected_boxes)

        # Draw bounding boxes and labels
        for obj_idx, bbox in tracked_objects.items():
            x1, y1, x2, y2 = map(int, bbox)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(frame, f'ID: {obj_idx}', (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
 
        # Display the resulting frame
        cv2.imshow('Frame', frame)

        # Break loop with 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Print final count
    #print(f"Total unique people detected: {len(tracked_objects)}")
    
    # When everything done, release the capture and destroy all windows
    cap.release()
    cv2.destroyAllWindows()
else:
    print("Webcam not found. Please check the connection and try again.")


