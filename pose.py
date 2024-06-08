#Program to create custom ArUco dictionary using OpenCV and detect markers using webcam
# original code from: http://www.philipzucker.com/aruco-in-opencv/
# Modified by Iyad Aldaqre
# 12.07.2019
import numpy as np
import cv2
import cv2.aruco as aruco
def rotation_vector_to_euler_angles(rvec,tvec):
    # Convert rotation vector to rotation matrix
    rotation_mat, _ = cv2.Rodrigues(rvec)
    pose_mat = cv2.hconcat((rotation_mat, tvec))
    _, _, _, _, _, _, euler_angles = cv2.decomposeProjectionMatrix(pose_mat)
    return euler_angles
# define an empty custom dictionary with 
aruco_dict = aruco.Dictionary(0, 5, 1)
# add empty bytesList array to fill with 3 markers later
aruco_dict.bytesList = np.empty(shape = (10, 4, 4), dtype = np.uint8)
# add new marker(s)
mybits = np.array([[1,0,0,0,0],[1,0,0,0,0],[0,1,0,0,1],[0,1,0,0,1],[1,0,0,0,0]], dtype = np.uint8)
aruco_dict.bytesList[0] = aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[1,0,0,0,0],[1,0,0,0,0],[0,1,1,1,0],[0,1,0,0,1],[0,1,0,0,1]], dtype = np.uint8)
aruco_dict.bytesList[1] = aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[1,0,0,0,0],[1,0,1,1,1],[1,0,0,0,0],[1,0,0,0,0],[0,1,1,1,0]], dtype = np.uint8)
aruco_dict.bytesList[2] = aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[1,0,0,0,0],[1,0,1,1,1],[1,0,0,0,0],[0,1,1,1,0],[0,1,0,0,1]], dtype = np.uint8)
aruco_dict.bytesList[3] = aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[1,0,0,0,0],[1,0,0,0,0],[0,1,0,0,1],[0,1,1,1,0],[1,0,0,0,0]], dtype = np.uint8)
aruco_dict.bytesList[4] = aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[1,0,1,1,1],[1,0,0,0,0],[0,1,0,0,1],[0,1,0,0,1],[1,0,1,1,1]], dtype = np.uint8)
aruco_dict.bytesList[5] = aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[1,0,1,1,1],[1,0,1,1,1],[1,0,1,1,1],[0,1,0,0,1],[1,0,0,0,0]], dtype = np.uint8)
aruco_dict.bytesList[6] = aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[1,0,0,0,0],[0,1,0,0,1],[1,0,1,1,1],[1,0,1,1,1],[0,1,0,0,1]], dtype = np.uint8)
aruco_dict.bytesList[7] = aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[1,0,1,1,1],[1,0,0,0,0],[1,0,0,0,0],[0,1,1,1,0],[1,0,1,1,1]], dtype = np.uint8)
aruco_dict.bytesList[8] = aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[1,0,1,1,1],[0,1,1,1,0],[1,0,0,0,0],[0,1,1,1,0],[1,0,0,0,0]], dtype = np.uint8)
aruco_dict.bytesList[9] = aruco.Dictionary_getByteListFromBits(mybits)
# save marker images
for i in range(len(aruco_dict.bytesList)):
    cv2.imwrite("custom_aruco_" + str(i) + ".png", aruco.generateImageMarker(aruco_dict, i, 128))
# open video capture from (first) webcam
cap = cv2.VideoCapture(0)
def pose_estimation(frame, aruco_dict, matrix_coefficients, distortion_coefficients):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    parameters = aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict,parameters)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is not None and len(ids) > 0:
        object_points = np.array([[0, 0, 0],
                                  [0, 1, 0],
                                  [1, 1, 0],
                                  [1, 0, 0]], dtype=np.float32)
        for i in range(len(ids)):
            _, rvecs, tvecs = cv2.solvePnP(object_points, corners[i][0],matrix_coefficients,distortion_coefficients)
            # Draw detected markers
            aruco.drawDetectedMarkers(frame, corners)
            # Draw coordinate axes
            axis_length = 0.1  # Length of the axis in meters
            axis_points = np.float32([[0,0,0], [axis_length,0,0], [0,axis_length,0], [0,0,-axis_length]]).reshape(-1,3)
            image_points, _ = cv2.projectPoints(axis_points, rvecs, tvecs, matrix_coefficients, distortion_coefficients)
            frame = cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvecs, tvecs, axis_length)
            frame = aruco.drawDetectedMarkers(frame, corners, ids)
            a=rotation_vector_to_euler_angles(rvecs,tvecs)
            if a[0][0]<0:
                print(180+a[0][0])
            else:
                print((180-a[0][0])*(-1))
    return frame
intrinsic_camera=np.array(((637.09581972,0,338.41902037),(0,636.92889535,244.41188742),(0,0,1)))
distortion=np.array((-0.20441599  ,1.03051155  ,0.00375628  ,0.00769952 ,-1.90159821))
while cap.isOpened():
    ret, img = cap.read()
    output = pose_estimation(img, aruco_dict, intrinsic_camera, distortion)
    cv2.imshow('Estimated Pose', output)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break