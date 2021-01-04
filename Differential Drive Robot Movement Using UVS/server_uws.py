import cv2
import sys
import numpy as np
from server import Server
from rectangle import Rectangle
from threading import Event, Thread
from time import sleep
from queue import Queue
from uvs import *


if __name__ == '__main__' :

    #Get OpenCV version
    #Start listening on port 9999
    server = Server(9999)

    #Thread-safe queue to get data from threads
    queue = Queue()
    #End effector KCF Tracker
    tracker, tracker_type = choose_tracking_method(2,3.6)

    second_tracker, second_tracker_type =  choose_tracking_method(2,3.6)

    #Initialize webcam
    vc = cv2.VideoCapture(0)

    #Get the first frame
    if vc.isOpened():
        rval, frame = vc.read()
    else:
        rval = False

    i = 0
    while rval and i < 100:
        # Read a new frame
        rval, frame = vc.read()
        i = i + 1

    #In here if rval is false we could throw an exception.

    cv2.namedWindow("webcam")


    #Select draw bounding boxes around end effector and target point
    bounding_rectangle, second_bounding_rectangle, target_bounding_rectangle = select_tracked_regions(vc)


    feature_point = bounding_rectangle.centre

    target_point = target_bounding_rectangle.centre

    second_point = second_bounding_rectangle.centre


    # Initialize tracker with first frame and bounding box
    #rval = tracker.init(frame, bounding_rectangle.array_representation)

    multiTracker = cv2.MultiTracker_create()
    multiTracker.add(tracker, frame, bounding_rectangle.array_representation)
    multiTracker.add(second_tracker, frame, second_bounding_rectangle.array_representation)


    ###############Estimate Initial Jacobian################################

    #r_height= 0.11
    #r_width = 0.18
    r_height = 0.18
    bbox_h = bounding_rectangle.height
    #bbox_w = bounding_rectangle.width
    ref_h = bbox_h/r_height
    #ref_w = bbox_w/r_width
    ref_w = ref_h
    pixel_to_meter = (ref_h+ref_w)/2
    error_vector = compute_delta(feature_point, target_point)
    while np.linalg.norm(error_vector) > 20:
        print(np.linalg.norm(error_vector))
        error_vector = isBetween(feature_point, target_point, second_point)
        while error_vector!= True:
            previous_feature_point = feature_point

            #tracking_failed, end_effector_bounding_box, second_bounding_box = move_and_track(multiTracker, vc, feature_point[0], feature_point[1], target_point, pixel_to_meter, server, queue)
            base = -2
            joint = 2
            tracking_failed, end_effector_bounding_box, second_bounding_box =  move_and_track(multiTracker, vc, base, joint, target_point, server, queue)
            if (tracking_failed):
                print("I am a fucking failure")
                rval, frame = vc.read()
                multiTracker = cv2.MultiTracker_create()
                tracker, tracker_type = choose_tracking_method(2,3.6)
                second_tracker, second_tracker_type =  choose_tracking_method(2,3.6)

                #rval = tracker.init(frame, end_effector_bounding_box.array_representation)
                multiTracker.add(tracker, frame, bounding_rectangle.array_representation)
                multiTracker.add(second_tracker, frame, second_bounding_rectangle.array_representation)



            feature_point = end_effector_bounding_box.centre
            second_point = second_bounding_box.centre

            error_vector = isBetween(feature_point, target_point, second_point)
        # server.sendTermination()
        # print("Done")
        error_vector = compute_delta(feature_point, target_point)
        inti_distance = np.linalg.norm(error_vector)+20
        while np.linalg.norm(error_vector) > 20:
            previous_feature_point = feature_point
            prev_err = np.linalg.norm(error_vector)+1
            #tracking_failed, end_effector_bounding_box, second_bounding_box = move_and_track(multiTracker, vc, feature_point[0], feature_point[1],second_point[0], second_point[1] target_point, pixel_to_meter, server, queue)
            base = 5
            joint = 5
            tracking_failed, end_effector_bounding_box, second_bounding_box =  move_and_track(multiTracker, vc, base, joint, target_point, server, queue)
            if (tracking_failed):
                rval, frame = vc.read()
                multiTracker = cv2.MultiTracker_create()
                tracker, tracker_type = choose_tracking_method(2,3.6)
                second_tracker, second_tracker_type =  choose_tracking_method(2,3.6)

                #rval = tracker.init(frame, end_effector_bounding_box.array_representation)
                multiTracker.add(tracker, frame, bounding_rectangle.array_representation)
                multiTracker.add(second_tracker, frame, second_bounding_rectangle.array_representation)




            feature_point = end_effector_bounding_box.centre
            second_point = second_bounding_box.centre

            error_vector = compute_delta(feature_point, target_point)
            print("Prev Error DISTANCE"+str(prev_err))
            print("Error DISTANCE"+str(np.linalg.norm(error_vector)))
            if(np.linalg.norm(error_vector)> prev_err ):
                print("IT IS Broken")
                break
        error_vector = compute_delta(feature_point, target_point)
    server.sendTermination()
    print("Done")
