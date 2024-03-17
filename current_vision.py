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
from   wpimath.geometry import *
from dt_apriltags import Detector
from concurrent.futures import ThreadPoolExecutor
import threading
import math
import traceback

from cscore import CameraServer, CvSink, CvSource, VideoSource, UsbCamera, MjpegServer
from ntcore import NetworkTableInstance, EventFlags
from robotpy_apriltag import AprilTagFieldLayout

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

robot_dist = 200
calibration_pose = (0, 114/100, (-robot_dist+41)/100, 1, 0.25, 0, 0)

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
camera_matrix = np.matrix(
    [
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 0],
    ]
)

i = 0
tagSize = 0.1651
            # tag_size=0.206375)
            # tag_size=0.06)
lock = threading.Lock()
field = AprilTagFieldLayout("2024-crescendo.json")

def startCamera(config):
    try:
        print("Starting camera '{}' on {}".format(config.name, config.path))
        NAME = config.name.replace(" ", "_")
        # if NAME in ["Top_Left"]:
        #     return

        camera = UsbCamera(config.name, config.path)
        global i;
        mjpegServer1 = MjpegServer("serve_"+NAME, 1181+i);
        i += 1;

        ntinst = NetworkTableInstance.getDefault()
        vision_nt = ntinst.getTable(NAME)
        vision_values = {}

        last_pose = {}
        last_rotation = {}

        server = CameraServer.startAutomaticCapture(camera=camera)
        cvSink = CvSink("Detection_cam_"+config.name.replace(" ", "_"));
        cvSink.setSource(camera);

        cvSource = CameraServer.putVideo("Corrected_cam_"+config.name.replace(" ", "_"), 1920, 1080);
        mjpegServer1.setSource(cvSource);

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

            def write_frame(frame):
                # cv2.imwrite("pictures/"+cvSink.getName() + ".jpg", frame)
                cvSource.putFrame(frame)

            detected = at_detector.detect(image_undistorted, estimate_tag_pose=True, camera_params=(fx, fy, cx, cy), tag_size=tagSize)
            if len(detected) > 0:
                lock.acquire()
                # print(f"Detected: {len(detected)}")

                def submit_tag(tag):
                    try:
                        def draw_pose_box(img, camera_matrix, pose, z_sign = 1):
                            """
                            Draws the 3d pose box around the AprilTag.

                            :param img: The image to write on.
                            :param camera_matrix: The camera's intrinsic calibration matrix.
                            :param pose: The ``Pose3d`` of the tag.
                            :param z_sign: The direction of the z-axis.
                            """
                            # Creates object points
                            opoints = np.array([
                                                -1, -1, 0,
                                                1, -1, 0,
                                                1,  1, 0,
                                                -1,  1, 0,
                                                -1, -1, -2 * z_sign,
                                                1, -1, -2 * z_sign,
                                                1,  1, -2 * z_sign,
                                                -1,  1, -2 * z_sign,
                                            ]).reshape(-1, 1, 3) * 0.5 * tagSize

                            # Creates edges
                            edges = np.array([
                                                0, 1,
                                                1, 2,
                                                2, 3,
                                                3, 0,
                                                0, 4,
                                                1, 5,
                                                2, 6,
                                                3, 7,
                                                4, 5,
                                                5, 6,
                                                6, 7,
                                                7, 4
                                            ]).reshape(-1, 2)

                            # Calulcates rotation and translation vectors for each AprilTag
                            rVecs, _ = cv2.Rodrigues(pose[:3,:3])
                            tVecs = pose[:3, 3:]

                            # Derivative coefficients
                            dcoeffs = np.zeros(5)

                            # Calulate image points of each AprilTag
                            ipoints, _ = cv2.projectPoints(opoints, rVecs, tVecs, camera_matrix, dcoeffs)
                            ipoints = np.round(ipoints).astype(int)
                            ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

                            # Draws lines between all the edges
                            for i, j in edges:
                                cv2.line(img, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)

                        def draw_pose_axes(img, camera_matrix, pose, center):
                            """
                            Draws the colored pose axes around the AprilTag.

                            :param img: The image to write on.
                            :param camera_matrix: The camera's intrinsic calibration matrix.
                            :param pose: The ``Pose3d`` of the tag.
                            :param center: The center of the AprilTag.
                            """
                            # Calulcates rotation and translation vectors for each AprilTag
                            rVecs, _ = cv2.Rodrigues(pose[:3,:3])
                            tVecs    = pose[:3, 3:]

                            # Derivative coefficients
                            dcoeffs = np.zeros(5)

                            # Calculate object points of each AprilTag
                            opoints = np.float32([[1, 0, 0],
                                                [0, -1, 0],
                                                [0, 0, -1]
                                                ]).reshape(-1, 3) * tagSize

                            # Calulate image points of each AprilTag
                            ipoints, _ = cv2.projectPoints(opoints, rVecs, tVecs, camera_matrix, dcoeffs)
                            ipoints = np.round(ipoints).astype(int)

                            # Calulates the center
                            center = np.round(center).astype(int)
                            center = tuple(center.ravel())

                            # Draws the 3d pose lines
                            cv2.line(img, center, tuple(ipoints[0].ravel()), (0, 0, 255), 2)
                            cv2.line(img, center, tuple(ipoints[1].ravel()), (0, 255, 0), 2)
                            cv2.line(img, center, tuple(ipoints[2].ravel()), (255, 0, 0), 2)
                        # Creates a 3d pose array from the rotation matrix and translation vectors
                        poseMatrix = np.concatenate([tag.pose_R, tag.pose_t], axis = 1)

                        # Draws varying levels of information onto the image
                        draw_pose_box(image_undistorted, camera_matrix, poseMatrix)
                        draw_pose_axes(image_undistorted, camera_matrix, poseMatrix, tag.center)
                        write_frame(image_undistorted)

                        # Calculate Pose3d
                        x, y, z = 0, 0, 0
                        pose = Pose3d()

                        # Extract the tag data from the detection results
                        if (poseMatrix is not None):
                            # Flattens the pose matrix into a 1D array
                            flatPose = np.array(poseMatrix).flatten()

                            # Creates the Pose3d components for a tag in the AprilTags WCS
                            try:
                                tempRot = Rotation3d(
                                    np.array([
                                        [flatPose[0], flatPose[1], flatPose[2]],
                                        [flatPose[4], flatPose[5], flatPose[6]],
                                        [flatPose[8], flatPose[9], flatPose[10]]
                                    ])
                                )
                            except ValueError as e:
                                print(e)
                                tempRot = Rotation3d()
                            tempTrans = Translation3d(flatPose[3], flatPose[7], flatPose[11])

                            # Get the camera's measured X, Y, and Z
                            tempX = tempTrans.Z()
                            y = -tempTrans.X()
                            z = -tempTrans.Y()

                            # Create a Rotation3d object
                            rot = Rotation3d(tempRot.Z(), -tempRot.X(), -tempRot.Y())

                            # Calulates the field relative X and Y coordinate
                            yTrans = Translation2d(tempX, y).rotateBy(Rotation2d(-rot.Z()))
                            x = yTrans.X()
                            y = yTrans.Y()

                            # Calulates the field relative Z coordinate
                            zTrans = Translation2d(tempX, z).rotateBy(Rotation2d(np.pi + rot.Y()))
                            z = zTrans.Y()

                            # Create a Translation3d object
                            trans = Translation3d(x, y, z)

                            # Creates a Pose3d object in the field WCS
                            pose = Pose3d(trans, rot)
                            transform = Transform3d(
                                trans, rot
                            )

                        print(pose)

                        vision_values[tag.tag_id].set([pose.x, pose.y, pose.z, rot.getQuaternion().W(), rot.getQuaternion().X(), rot.getQuaternion().Y(), rot.getQuaternion().Z()], frame_time)

                        targetPose = field.getTagPose(tag.tag_id)

                        cameraToTag = Transform3d(
                            trans,
                            Rotation3d(
                                rot.X(),
                                rot.Y(),
                                math.pi-rot.Z()
                            )
                        )

                        camPose = targetPose.transformBy(cameraToTag)

                        rot = camPose.rotation()
                        vision_values[str(tag.tag_id)+"_debugtag"].set([
                            camPose.x, camPose.y, camPose.z, rot.getQuaternion().W(), rot.getQuaternion().X(), rot.getQuaternion().Y(), rot.getQuaternion().Z()
                        ], frame_time)
                    except Exception:
                        traceback.print_exc()

                for tag in detected:
                    # print(f"{tag.tag_id} Position: {tag.pose_t}")
                    # print(f"{tag.tag_id} Rotation: {tag.pose_R}")

                    # tag.pose_t = np.multiply(tag.pose_t, 100) # convert to cm

                    # print(f"[{NAME}]  Tag {tag.tag_id} Height: {tag.pose_t[1]}")

                    if tag.tag_id not in last_pose:
                        print(tag.pose_t)
                        last_pose[tag.tag_id] = tag.pose_t
                        last_rotation[tag.tag_id] = tag.pose_R
                        vision_values[tag.tag_id] = vision_nt.getDoubleArrayTopic("tag_"+str(tag.tag_id)).publish()
                        vision_values[str(tag.tag_id)+"_debugtag"] = vision_nt.getDoubleArrayTopic("debug_"+str(tag.tag_id)).publish()
                        submit_tag(tag)
                        continue
                    submit_tag(tag)

                    # if np.linalg.norm(tag.pose_t - last_pose[tag.tag_id]) > 0.1:
                    #     print(f"[{NAME}]  Tag {tag.tag_id} moved too much")
                    #     print(f"[{NAME}]  Move Distance: {np.linalg.norm(tag.pose_t - last_pose[tag.tag_id])}")

                    # if np.linalg.norm(tag.pose_R - last_rotation[tag.tag_id]) > 0.1:
                    #     print(f"[{NAME}]  Tag {tag.tag_id} rotated too much")
                    #     print(f"[{NAME}]  Rotate Distance: {np.linalg.norm(tag.pose_R - last_rotation[tag.tag_id])}")

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
        # ntinst.setServer("172.16.0.91")
        ntinst.startClient4("wpilibpi")
        ntinst.setServerTeam(team)
        ntinst.startDSClient()

    CameraServer.setSize(CameraServer.kSize160x120)
    with ThreadPoolExecutor(max_workers=4) as exe:
        for cameraConfig in cameraConfigs:
            # startCamera(cameraConfig)
            exe.submit(startCamera, cameraConfig)
    print("Done")