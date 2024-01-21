#!/usr/bin/env python3

# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import json
import time
import sys
import traceback

import mrcal
import cv2
import numpy as np
from dt_apriltags import Detector
from concurrent.futures import ThreadPoolExecutor
import threading
import math

from cscore import CameraServer, CvSink, VideoSource, UsbCamera, MjpegServer
from ntcore import NetworkTableInstance, EventFlags

read_only = False
try:
    with open("a", "w") as f:
        pass
except:
    read_only = True

#   JSON format:
#   {
#       "team": <team number>,
#       "ntmode": <"client" or "server", "client" if unspecified>
#       "cameras": [
#           {
#               "name": <camera name>
#               "path": <path, e.g. "/dev/video0">
#               "pixel format": <"MJPEG", "YUYV", etc>   // optional
#               "width": <video mode width>              // optional
#               "height": <video mode height>            // optional
#               "fps": <video mode fps>                  // optional
#               "brightness": <percentage brightness>    // optional
#               "white balance": <"auto", "hold", value> // optional
#               "exposure": <"auto", "hold", value>      // optional
#               "properties": [                          // optional
#                   {
#                       "name": <property name>
#                       "value": <property value>
#                   }
#               ],
#               "stream": {                              // optional
#                   "properties": [
#                       {
#                           "name": <stream property name>
#                           "value": <stream property value>
#                       }
#                   ]
#               }
#           }
#       ]
#       "switched cameras": [
#           {
#               "name": <virtual camera name>
#               "key": <network table key used for selection>
#               // if NT value is a string, it's treated as a name
#               // if NT value is a double, it's treated as an integer index
#           }
#       ]
#   }

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []
cameras = []

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    return True

model_orig = mrcal.cameramodel("camera-0.cameramodel")
model_pinhole = mrcal.pinhole_model_for_reprojection(model_orig)

mapxy = mrcal.image_transformation_map(model_orig, model_pinhole, mask_valid_intrinsics_region_from=True)
fx, fy, cx, cy = model_orig.intrinsics()[1][0:4]

i = 0
lock = threading.Lock()

# https://github.com/blender/blender/blob/756538b4a117cb51a15e848fa6170143b6aafcd8/source/blender/blenlib/intern/math_rotation.c#L272
def matrixToQuaternion(mat):
    q = [0, 0, 0, 0]
    if (mat[2][2] < 0):
        if (mat[0][0] > mat[1][1]):
            trace = 1 + mat[0][0] - mat[1][1] - mat[2][2];
            s = 2 * math.sqrt(trace);
            if (mat[1][2] < mat[2][1]):
                s = -s;
            q[1] = 0.25 * s;
            s = 1 / s;
            q[0] = (mat[1][2] - mat[2][1]) * s;
            q[2] = (mat[0][1] + mat[1][0]) * s;
            q[3] = (mat[2][0] + mat[0][2]) * s;
            if (trace == 1) and (q[0] == 0 and q[2] == 0 and q[3] == 0):
                q[1] = 1;
        else:
            trace = 1 - mat[0][0] + mat[1][1] - mat[2][2];
            s = 2 * sqrtf(trace);
            if (mat[2][0] < mat[0][2]):
                s = -s;
            q[2] = 0.25* s;
            s = 1.0 / s;
            q[0] = (mat[2][0] - mat[0][2]) * s;
            q[1] = (mat[0][1] + mat[1][0]) * s;
            q[3] = (mat[1][2] + mat[2][1]) * s;
            if (trace == 1.0) and (q[0] == 0 and q[1] == 0 and q[3] == 0):
                q[2] = 1;
    else:
        if (mat[0][0] < -mat[1][1]):
            trace = 1 - mat[0][0] - mat[1][1] + mat[2][2];
            s = 2 * math.sqrt(trace);
            if (mat[0][1] < mat[1][0]):
                s = -s;
            q[3] = 0.25 * s;
            s = 1.0 / s;
            q[0] = (mat[0][1] - mat[1][0]) * s;
            q[1] = (mat[2][0] + mat[0][2]) * s;
            q[2] = (mat[1][2] + mat[2][1]) * s;
            if (trace == 1) and (q[0] == 0 and q[1] == 0 and q[2] == 0):
                q[3] = 1
        else:
            trace = 1 + mat[0][0] + mat[1][1] + mat[2][2];
            s = 2.0 * math.sqrt(trace);
            q[0] = 0.25 * s;
            s = 1.0 / s;
            q[1] = (mat[1][2] - mat[2][1]) * s;
            q[2] = (mat[2][0] - mat[0][2]) * s;
            q[3] = (mat[0][1] - mat[1][0]) * s;
            if (trace == 1) and (q[1] == 0 and q[2] == 0 and q[3] == 0):
                q[0] = 1;
    return q

def startCamera(config):
    try:
        print("Starting camera '{}' on {}".format(config.name, config.path))
        NAME = config.name.replace(" ", "_")
        if NAME in ["Top_Left"]:
            return

        camera = UsbCamera(config.name, config.path)
        global i;
        mjpegServer1 = MjpegServer("serve_"+NAME, 1181+i);
        i += 1;
        mjpegServer1.setSource(camera);

        ntinst = NetworkTableInstance.getDefault()
        vision_nt = ntinst.getTable(NAME)
        vision_values = {}
        
        last_pose = {}
        last_rotation = {}
        

        server = CameraServer.startAutomaticCapture(camera=camera)
        cvSink = CvSink("Detection_cam_"+config.name.replace(" ", "_"));
        cvSink.setSource(camera);

        camera.setConfigJson(json.dumps(config.config))
        camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)
        
        img = np.zeros(shape=(1280, 720, 3), dtype=np.uint8)
        at_detector = Detector(searchpath=['apriltags'],
                        families='tag36h11',
                        nthreads=1,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0)
        
        while True:
            # print("Starting capture: "+cvSink.getName())
            
            frame_time, img = cvSink.grabFrame(img)
            # print("Frame time: "+str(frame_time))
            if frame_time == 0:
                print("Error: "+str(cvSink.getError()))
                time.sleep(2)
                continue

            bw = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            image_undistorted = mrcal.transform_image(bw, mapxy, interpolation=cv2.INTER_LINEAR)

            # cv2.imwrite("pictures/"+cvSink.getName() + ".jpg", image_undistorted)

            detected = at_detector.detect(image_undistorted, estimate_tag_pose=True, camera_params=(fx, fy, cx, cy), 
            # tag_size=0.206375)
            tag_size=0.06)
            if len(detected) > 0:
                lock.acquire()
                # print(f"Detected: {len(detected)}")
                for tag in detected:
                    if tag.tag_id > 5:
                        continue

                    # print(f"{tag.tag_id} Position: {tag.pose_t}")
                    # print(f"{tag.tag_id} Rotation: {tag.pose_R}")

                    print(f"[{NAME}]  Tag {tag.tag_id} Height: {tag.pose_t[1]}")

                    quat = matrixToQuaternion(tag.pose_R)
                    if tag.tag_id not in last_pose:
                        last_pose[tag.tag_id] = tag.pose_t
                        last_rotation[tag.tag_id] = tag.pose_R
                        vision_values[tag.tag_id] = vision_nt.getDoubleArrayTopic("tag_"+str(tag.tag_id)).publish()
                        print(tag.pose_R)
                        vision_values[tag.tag_id].set([tag.pose_t[0], tag.pose_t[1], tag.pose_t[2], quat[0], quat[1], quat[2], quat[3]], frame_time)
                        continue
                    vision_values[tag.tag_id].set([tag.pose_t[0], tag.pose_t[1], tag.pose_t[2], quat[0], quat[1], quat[2], quat[3]], frame_time)
                    
                    if np.linalg.norm(tag.pose_t - last_pose[tag.tag_id]) > 0.1:
                        print(f"[{NAME}]  Tag {tag.tag_id} moved too much")
                        print(f"[{NAME}]  Move Distance: {np.linalg.norm(tag.pose_t - last_pose[tag.tag_id])}")

                    if np.linalg.norm(tag.pose_R - last_rotation[tag.tag_id]) > 0.1:
                        print(f"[{NAME}]  Tag {tag.tag_id} rotated too much")
                        print(f"[{NAME}]  Rotate Distance: {np.linalg.norm(tag.pose_R - last_rotation[tag.tag_id])}")

                    last_pose[tag.tag_id] = tag.pose_t
                    last_rotation[tag.tag_id] = tag.pose_R
                # time.sleep(0.2)
                lock.release()
    except:
        print("Error: "+str(traceback.format_exc()))


if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    # start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.setServer("172.16.0.91")
        ntinst.startClient4("wpilibpi")
        # ntinst.setServerTeam(team)
        ntinst.startDSClient()

    CameraServer.setSize(CameraServer.kSize160x120)
    with ThreadPoolExecutor(max_workers=4) as exe:
        for cameraConfig in cameraConfigs:
            # startCamera(cameraConfig)
            exe.submit(startCamera, cameraConfig)
    print("Done")
