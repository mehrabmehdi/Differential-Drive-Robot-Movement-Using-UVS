import cv2
import sys
import numpy as np
from server import Server
from rectangle import Rectangle
from threading import Event, Thread
from time import sleep
from queue import Queue




#Choosing tracking method out all of the possible implementations provided by the OpenCV contrib module

def choose_tracking_method(index,minor_ver):
    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
    tracker_type = tracker_types[index]

    if tracker_type == 'BOOSTING':
        tracker = cv2.TrackerBoosting_create()
    if tracker_type == 'MIL':
        tracker = cv2.TrackerMIL_create()
    if tracker_type == 'KCF':
        tracker = cv2.TrackerKCF_create()
    if tracker_type == 'TLD':
        tracker = cv2.TrackerTLD_create()
    if tracker_type == 'MEDIANFLOW':
        tracker = cv2.TrackerMedianFlow_create()
    if tracker_type == 'GOTURN':
        tracker = cv2.TrackerGOTURN_create()
    if tracker_type == 'MOSSE':
        tracker = cv2.TrackerMOSSE_create()
    if tracker_type == "CSRT":
        tracker = cv2.TrackerCSRT_create()
    return tracker, tracker_type

#Computes the delta of two vectors, where each vector is a tuple in image space (u,v)
def compute_delta(initial_vector, final_vector):
    return (final_vector[0] - initial_vector[0], final_vector[1] - initial_vector[1])


#
# def move_and_track(multiTracker, vc, robot_x, robot_y, second_x, second_y, target_point, pixel_to_meter ,server, queue):
#     target_x = target_point[0]/pixel_to_meter
#     target_y = target_point[1]/pixel_to_meter
#     robot_x = robot_x/pixel_to_meter
#     robot_y = robot_y/pixel_to_meter
#     second_x = second_x/pixel_to_meter
#     second_y = second_y/pixel_to_meter
#     robot_movement_thread = Thread(target=server.sendCoords, args=(robot_x, robot_y, target_x, target_y, queue))

def move_and_track(multiTracker, vc,  base_angle, joint_angle, target_point, server, queue):
    robot_movement_thread = Thread(target=server.sendAngles, args=(base_angle,joint_angle, queue))
    robot_movement_thread.start()
    tracking_failed = False

    while robot_movement_thread.is_alive():
        rval, frame = vc.read()
        ok, bbox = multiTracker.update(frame)
        bounding_rectangle = Rectangle(bbox[0])
        second_bounding_rectangle = Rectangle(bbox[1])
        if ok:
            current_position = bounding_rectangle.centre
            second_point = second_bounding_rectangle.centre
            cv2.rectangle(frame, bounding_rectangle.top_left, bounding_rectangle.bottom_right, (255,0,0), 2, 1)
            cv2.rectangle(frame, (int(current_position[0]) - 2, int(current_position[1]) - 2), (int(current_position[0]) + 2,  int(current_position[1]) + 2), (0, 0, 0), -1)
            cv2.putText(frame, "Feature Point (x,y): "  + str(current_position), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            cv2.rectangle(frame, second_bounding_rectangle.top_left, second_bounding_rectangle.bottom_right, (255,0,0), 2, 1)
            cv2.rectangle(frame, (int(second_point[0]) - 2, int(second_point[1]) - 2), (int(second_point[0]) + 2,  int(second_point[1]) + 2), (0, 0, 0), -1)
            cv2.putText(frame, "Second Point (x,y): "  + str(second_point), (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            cv2.putText(frame, "Target Point (x,y): "  + str(target_point), (100,110), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            cv2.rectangle(frame, (int(target_point[0]) - 2, int(target_point[1]) - 2), (int(target_point[0]) + 2,  int(target_point[1]) + 2), (0, 128, 255), -1)
            tracking_failed = False
        else :
            tracking_failed = True

        cv2.imshow("webcam", frame)
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break
    #If we lost tracking and couldn't recover, we need the user to manually select the end effector:
    if tracking_failed:
        print("Failure")
        bounding_rectangle, second_bounding_rectangle, target_bounding_rectangle = select_tracked_regions(vc)

    return tracking_failed, bounding_rectangle, second_bounding_rectangle



def select_tracked_regions(vc):

    rval, frame = vc.read()
    cv2.putText(frame, "Select Robot center", (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 127, 0), 2)

    bbox = cv2.selectROI("webcam",frame, False)

    bounding_rectangle = Rectangle(bbox)

    feature_point = bounding_rectangle.centre
    rval, frame = vc.read()
    cv2.rectangle(frame, bounding_rectangle.top_left, bounding_rectangle.bottom_right, (255,0,0), 2, 1)
    cv2.rectangle(frame, (int(feature_point[0]) - 2, int(feature_point[1]) - 2), (int(feature_point[0]) + 2,  int(feature_point[1]) + 2), (0, 0, 0), -1)

    cv2.putText(frame, "Feature Point (x,y): "  + str(feature_point), (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
    #Display prompt to select target point
    cv2.putText(frame, "Select Robot Head", (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 127, 0), 2)
    second_bbox = cv2.selectROI("webcam", frame, False)
    #Convert the target bounding box into a Rectangle object to perform automatic calculation of the four corners as well as the centre point
    second_bounding_rectangle = Rectangle(second_bbox)
    second_point = second_bounding_rectangle.centre
    rval, frame = vc.read()
    cv2.rectangle(frame, second_bounding_rectangle.top_left, second_bounding_rectangle.bottom_right, (255,0,0), 2, 1)
    cv2.rectangle(frame, (int(second_point[0]) - 2, int(second_point[1]) - 2), (int(second_point[0]) + 2,  int(second_point[1]) + 2), (0, 0, 0), -1)

    cv2.putText(frame, "Second Point (x,y): "  + str(second_point), (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

    #Display prompt to select target point
    cv2.putText(frame, "Select Target Point", (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 127, 0), 2)
    target_bbox = cv2.selectROI("webcam", frame, False)
    #Convert the target bounding box into a Rectangle object to perform automatic calculation of the four corners as well as the centre point
    target_bounding_rectangle = Rectangle(target_bbox)
    return bounding_rectangle, second_bounding_rectangle, target_bounding_rectangle

def isBetween(a, b, c):
    crossproduct = (c[1] - a[1]) * (b[0] - a[0]) - (c[0] - a[0]) * (b[1]- a[1])

    # compare versus epsilon for floating point values, or != 0 if using integers
    print( "THE crossproduct is : "+str(crossproduct))
    if abs(crossproduct) > 200:
        return False

    dotproduct = (c[0] - a[0]) * (b[0] - a[0]) + (c[1] - a[1])*(b[1] - a[1])
    if dotproduct < 0:
        return False

    squaredlengthba = (b[0] - a[0])*(b[0] - a[0]) + (b[1] - a[1])*(b[1] - a[1])
    if dotproduct > squaredlengthba:
        return False

    return True
