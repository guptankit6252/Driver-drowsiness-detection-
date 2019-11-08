# importing the necessary packages for the project 
import numpy as np
import playsound
import dlib
import cv2
import argparse
import imutils
import time
import serial
from imutils import face_utils
from scipy.spatial import distance as dist
from imutils.video import VideoStream
from threading import Thread
Arduino=serial.Serial('/dev/ttyACM0',1)
#time to connect arduino
time.sleep(1)

#function to calculate the EAR
def calculate_EAR(eye):
	# calculating the euclidian distance between vertical eye landmark
	A = dist.euclidean(eye[1], eye[5])
	B = dist.euclidean(eye[2], eye[4])

	# calculating the euclidean distance between horizontal eye landmark
	C = dist.euclidean(eye[0], eye[3])

	# formula for calculating EAR
	EAR = (A + B) / (2.0 * C)


	return EAR

#function to sound the alarm
def play_sound(path):
	playsound.playsound(path)

# arguments to be given at the time of running the code
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--shape-predictor", required=True,
	help="path to facial landmark predictor")
ap.add_argument("-a", "--alarm", type=str, default="",
	help="path alarm .WAV file")
ap.add_argument("-w", "--webcam", type=int, default=0,
	help="index of webcam on system")
args = vars(ap.parse_args())

# these are two constants to describe about the threshold EAR and threshold value total number of consecutive frames for which EAR becomes less than the threshold value 
EAR_threshold = 0.25
EAR_consec_frames = 50

# initializing counter for consecutive frames and boolean for alarm
COUNTER = 0
ALARM_ON = False

# initialize dlib's face detector (HOG-based) and then create the facial landmark predictor
print("[INFO] loading facial landmark predictor...")
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(args["shape_predictor"])

# grab the indexes of the facial landmarks for the left and
# right eye, respectively
(left_Start, left_End) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
(right_Start, right_End) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]

# start the video stream thread
print("[INFO] starting video stream thread...")
vs = VideoStream(src=args["webcam"]).start()
time.sleep(1.0)

# loop over frames from the video stream
while True:
	# detecting frames and converting them to gray scale
        frame = vs.read()
        frame = imutils.resize(frame, width=450)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# detect faces in the grayscale frame
        rects = detector(gray, 0)

    	# loop over the face detections
        for (rect) in rects:
		# computing facial landmarks and then converting them to numpy array
                shape = predictor(gray, rect)
                shape = face_utils.shape_to_np(shape)

        # draw rectangle overthe detected face
                (x, y, l, h) = face_utils.rect_to_bb(rect)
                cv2.rectangle(frame, (x, y), (x + l, y + h), (58,39,152), 5)

        # loop to draw facial landmarks
                for (x, y) in shape:
                    cv2.circle(frame, (x, y), 1, (191,126,14), -1)
                    

		# calculating left EAR and right EAR
                leftEye  = shape[left_Start:left_End]
                rightEye = shape[right_Start:right_End]
                leftEAR  = calculate_EAR(leftEye)
                rightEAR = calculate_EAR(rightEye)

                EAR = (leftEAR + rightEAR) / 2.0

    		# drawing contours over the eyes
                leftEyeHull = cv2.convexHull(leftEye)
                rightEyeHull = cv2.convexHull(rightEye)
                cv2.drawContours(frame, [leftEyeHull], -1, (0,250,154), 1)
                cv2.drawContours(frame, [rightEyeHull], -1, (0,250,154), 1)

                if EAR < EAR_threshold:
                    COUNTER += 1

                    if COUNTER >= EAR_consec_frames:
				# alarm becomes on if it is off
                                if not ALARM_ON:
                                    ALARM_ON = True

					# checking if the alarm file is supplied and if so then play the alarm
                                    if args["alarm"] != "":
                                            t = Thread(target=play_sound,args=(args["alarm"],))
                                            t.deamon = True
                                            t.start()
					#passing the value to write on the arduino
                                            Arduino.write(b'1')
                                            time.sleep(1)
                                        
				# displaying alert message
                                cv2.putText(frame, "DROWSINESS ALERT!", (10, 30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

		# if counter is less than EAR_consec_frames than it is a blink so restart with initializing the counter to zero
                else:
                    COUNTER = 0
                    Arduino.write(b'0')
                    time.sleep(0.01)
                    ALARM_ON = False

       		# drawing the EAR on the screen
                    cv2.putText(frame, "EAR: {:.2f}".format(EAR), (300, 30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
 
	# show the frame
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
 
	# if the `q` key was pressed, break from the loop
        if key == ord("e"):
             break
 
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
