#!/usr/bin/env python3
import cv2
import depthai as dai
import queue
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
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.initialConfig.setMedianFilter(median)

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

camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setIspScale(2, 3)
camRgb.setFps(desired_fps)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
camRgb.initialControl.setManualFocus(130)

# Create XLink output
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("frames")
xout_depth = pipeline.create(dai.node.XLinkOut)
xout_depth.setStreamName("depth")

# Stream all the camera streams through the same XLink node
camRgb.isp.link(xout.input)
stereo.syncedLeft.link(xout.input)
stereo.syncedRight.link(xout.input)
stereo.depth.link(xout_depth.input)

q = queue.Queue()

def newFrame(inFrame):
    global q
    # Get "stream name" from the instance number
    num = inFrame.getInstanceNum()
    name = "num_"+str(num)
    if num == 0:
        print(fps())
    frame = inFrame.getCvFrame()
    # This is a different thread and you could use it to
    # run image processing algorithms here
    q.put({"name": name, "frame": frame})

def newDepthFrame(inFrame):
    global q
    frame = inFrame.getCvFrame()
    q.put({"name": "depth", "frame": frame})

# Connect to device and start pipeline
with dai.Device(pipeline, dai.DeviceInfo("192.168.122.111")) as device:

    # Add callback to the output queue "frames" for all newly arrived frames (color, left, right)
    device.getOutputQueue(name="frames", maxSize=4, blocking=False).addCallback(newFrame)
    device.getOutputQueue(name="depth", maxSize=4, blocking=False).addCallback(newDepthFrame)

    while True:
        # You could also get the data as non-blocking (block=False)
        data = q.get(block=True)
        cv2.imshow(data["name"], data["frame"])

        if cv2.waitKey(1) == ord('q'):
            break

