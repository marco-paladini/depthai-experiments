#!/usr/bin/env python3
import json

import cv2
import depthai as dai
import open3d as o3d
import time
import collections


class FPS:
    def __init__(self, avarageof=50):
        self.frametimestamps = collections.deque(maxlen=avarageof)
    def __call__(self):
        self.frametimestamps.append(time.time())
        if(len(self.frametimestamps) > 1):
            return "{:.2f} fps".format(len(self.frametimestamps)/(self.frametimestamps[-1]-self.frametimestamps[0]))
        else:
            return "0.0 fps"


fps = FPS()
COLOR = True
DISPLAY = True

lrcheck = True  # Better handling for occlusions
extended = False  # Closer-in minimum depth, disparity range is doubled
subpixel = True  # Better accuracy for longer distance, fractional disparity 32-levels
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7
median = dai.StereoDepthProperties.MedianFilter.KERNEL_7x7
desired_fps = 5

print("StereoDepth config options:")
print("    Left-Right check:  ", lrcheck)
print("    Extended disparity:", extended)
print("    Subpixel:          ", subpixel)
print("    Median filtering:  ", median)
print("    FPS:               ", desired_fps)
pipeline = dai.Pipeline()

monoLeft = pipeline.create(dai.node.MonoCamera)
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoLeft.setFps(desired_fps)

monoRight = pipeline.create(dai.node.MonoCamera)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
monoRight.setFps(desired_fps)

stereo = pipeline.createStereoDepth()
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.initialConfig.setMedianFilter(median)
# stereo.initialConfig.setConfidenceThreshold(255)

stereo.setLeftRightCheck(lrcheck)
stereo.setExtendedDisparity(extended)
stereo.setSubpixel(subpixel)
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

config = stereo.initialConfig.get()
config.postProcessing.speckleFilter.enable = False
config.postProcessing.speckleFilter.speckleRange = 50
config.postProcessing.temporalFilter.enable = True
config.postProcessing.spatialFilter.enable = True
config.postProcessing.spatialFilter.holeFillingRadius = 2
config.postProcessing.spatialFilter.numIterations = 1
config.postProcessing.thresholdFilter.minRange = 400
config.postProcessing.thresholdFilter.maxRange = 200000
config.postProcessing.decimationFilter.decimationFactor = 1
stereo.initialConfig.set(config)

xout_depth = pipeline.createXLinkOut()
xout_depth.setStreamName("depth")
stereo.depth.link(xout_depth.input)

# xout_disparity = pipeline.createXLinkOut()
# xout_disparity.setStreamName('disparity')
# stereo.disparity.link(xout_disparity.input)

xout_colorize = pipeline.createXLinkOut()
xout_colorize.setStreamName("colorize")
xout_rect_left = pipeline.createXLinkOut()
xout_rect_left.setStreamName("rectified_left")
xout_rect_right = pipeline.createXLinkOut()
xout_rect_right.setStreamName("rectified_right")
xout_sync_left = pipeline.createXLinkOut()
xout_sync_left.setStreamName("sync_left")
xout_sync_right = pipeline.createXLinkOut()
xout_sync_right.setStreamName("sync_right")

if COLOR:
    camRgb = pipeline.create(dai.node.ColorCamera)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setIspScale(2, 3)
    camRgb.setFps(desired_fps)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    camRgb.initialControl.setManualFocus(130)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    camRgb.isp.link(xout_colorize.input)
else:
    stereo.rectifiedRight.link(xout_colorize.input)

stereo.rectifiedLeft.link(xout_rect_left.input)
stereo.rectifiedRight.link(xout_rect_right.input)
stereo.syncedLeft.link(xout_sync_left.input)
stereo.syncedRight.link(xout_sync_right.input)


class HostSync:
    def __init__(self):
        self.arrays = {}

    def add_msg(self, name, msg):
        if not name in self.arrays:
            self.arrays[name] = []
        # Add msg to array
        self.arrays[name].append({"msg": msg, "seq": msg.getSequenceNum()})

        synced = {}
        for name, arr in self.arrays.items():
            for i, obj in enumerate(arr):
                if msg.getSequenceNum() == obj["seq"]:
                    synced[name] = obj["msg"]
                    break
        # If there are 5 (all) synced msgs, remove all old msgs
        # and return synced msgs
        if len(synced) >= 6:
            # Remove old msgs
            for name, arr in self.arrays.items():
                for i, obj in enumerate(arr):
                    if obj["seq"] < msg.getSequenceNum():
                        arr.remove(obj)
                    else:
                        break
            return synced
        return False


with dai.Device(pipeline, dai.DeviceInfo("192.168.122.111")) as device:
    serial_no = device.getMxId()
    print(f'Is EEPROM available: {device.isEepromAvailable()}')

    # User calibration
    try:
        with open(f"{serial_no}_user_calibration.json", "w") as outfile:
            json.dump(device.readCalibration2().eepromToJson(), outfile, indent=2)
    except Exception as ex:
        print(f'No user calibration: {ex}')

    # Factory calibration
    try:
        with open(f"{serial_no}_factory_calibration.json", "w") as outfile:
            json.dump(device.readFactoryCalibration().eepromToJson(), outfile, indent=2)
    except Exception as ex:
        print(f'No factory calibration: {ex}')

    with open(f"{serial_no}_raw_user_calibration.json", "w") as outfile:
        json.dump(device.readCalibrationRaw(), outfile)
    with open(f"{serial_no}_raw_factory_calibration.json", "w") as outfile:
        json.dump(device.readFactoryCalibrationRaw(), outfile)

    device.setIrLaserDotProjectorBrightness(1200)
    qs = []
    qs.append(device.getOutputQueue("depth", maxSize=1, blocking=False))
    qs.append(device.getOutputQueue("colorize", maxSize=1, blocking=False))
    qs.append(device.getOutputQueue("rectified_left", maxSize=1, blocking=False))
    qs.append(device.getOutputQueue("rectified_right", maxSize=1, blocking=False))
    qs.append(device.getOutputQueue("sync_left", maxSize=1, blocking=False))
    qs.append(device.getOutputQueue("sync_right", maxSize=1, blocking=False))

    try:
        from projector_3d import PointCloudVisualizer
    except ImportError as e:
        raise ImportError(
            f"\033[1;5;31mError occured when importing PCL projector: {e}. Try disabling the point cloud \033[0m "
        )

    calibData = device.readCalibration()
    if COLOR:
        w, h = camRgb.getIspSize()
        intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.RGB, dai.Size2f(w, h))
    else:
        w, h = monoRight.getResolutionSize()
        intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT, dai.Size2f(w, h))
    pcl_converter = PointCloudVisualizer(intrinsics, w, h, display=DISPLAY)

    sync = HostSync()
    depth_vis, color, rect_left, rect_right = None, None, None, None
    start_time = time.time()

    while True:
        for q in qs:
            new_msg = q.tryGet()
            if new_msg is not None:
                msgs = sync.add_msg(q.getName(), new_msg)
                if msgs:
                    print(fps())
                    depth = msgs["depth"].getFrame()
                    color = msgs["colorize"].getCvFrame()
                    rectified_left = msgs["rectified_left"].getCvFrame()
                    rectified_right = msgs["rectified_right"].getCvFrame()
                    sync_left = msgs["sync_left"].getCvFrame()
                    sync_right = msgs["sync_right"].getCvFrame()
                    depth_vis = cv2.normalize(depth, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
                    depth_vis = cv2.equalizeHist(depth_vis)
                    depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_HOT)
                    rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
                    pcl_converter.rgbd_to_projection(depth, rgb)
                    if DISPLAY:
                        cv2.imshow("depth", depth_vis)
                        cv2.imshow("color", color)
                        cv2.imshow("rectified_left", rectified_left)
                        cv2.imshow("rectified_right", rectified_right)
                        cv2.imshow("sync_left", sync_left)
                        cv2.imshow("sync_right", sync_right)
                        pcl_converter.visualize_pcd()

        elapsed = time.time() - start_time
        key = 0
        if not DISPLAY and elapsed > 10:
            key = ord("s")
        if DISPLAY:
            key = cv2.waitKey(10)
        if key == ord("s"):
            timestamp = str(int(time.time()))
            cv2.imwrite(f"{serial_no}_{timestamp}_depth.png", depth_vis)
            cv2.imwrite(f"{serial_no}_{timestamp}_color.png", color)
            cv2.imwrite(f"{serial_no}_{timestamp}_rectified_left.png", rectified_left)
            cv2.imwrite(f"{serial_no}_{timestamp}_rectified_right.png", rectified_right)
            cv2.imwrite(f"{serial_no}_{timestamp}_sync_left.png", sync_left)
            cv2.imwrite(f"{serial_no}_{timestamp}_sync_right.png", sync_right)
            o3d.io.write_point_cloud(f"{serial_no}_{timestamp}.pcd", pcl_converter.pcl, compressed=True)
            if not DISPLAY:
                print("done")
                break
        elif key == ord("q"):
           break
