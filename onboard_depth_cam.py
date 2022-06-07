#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
from scanner import Scanner

class OnboardDepthCam(object):
    def __init__(self, extended_disparity=False, subpixel=False, lr_check=True):
        self.pipeline = self.get_pipeline()
        self.device = None
        self.queue = None

    def __enter__(self):
        self.device = dai.Device(self.pipeline)
        self.device.__enter__()
        self.queue = self.device.getOutputQueue(name="disparity", maxSize=2, blocking=False)

    def __exit__(self, a, b, c):
        self.device.__exit__(a, b, c)
        self.device = None
        self.queue = None

    def get_pipeline(self, extended_disparity=False, subpixel=False, lr_check=True):
        # Create pipeline
        pipeline = dai.Pipeline()

        # Define sources and outputs
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        depth = pipeline.create(dai.node.StereoDepth)
        xout = pipeline.create(dai.node.XLinkOut)

        xout.setStreamName("disparity")

        # Properties
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
        depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
        depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        depth.setLeftRightCheck(lr_check)
        depth.setExtendedDisparity(extended_disparity)
        depth.setSubpixel(subpixel)

        # Linking
        monoLeft.out.link(depth.left)
        monoRight.out.link(depth.right)
        depth.disparity.link(xout.input)

        self.max_disparity = depth.initialConfig.getMaxDisparity()
        
        return pipeline

    def get_frame(self):
        if self.device is None:
            return None
        return self.queue.get().getFrame()

    def normalize_frame(self, frame):
        frame = (frame * (255 / self.max_disparity)).astype(np.uint8)
        return frame

min_val = 65
max_val = 85

def on_change_min(val):
    global min_val
    min_val = val

def on_change_max(val):
    global max_val
    max_val = val

def main():
    step = 45
    ext_disp = True
    subpix = False
    lr_check = True
    cam = OnboardDepthCam(extended_disparity=ext_disp, subpixel=subpix, lr_check=lr_check)
    scan = Scanner(-75, -60, 1280, 720, 850)

    # cv2.namedWindow("disparity")
    # cv2.createTrackbar('min', "disparity", 0, 100, on_change_min)
    # cv2.createTrackbar('max', "disparity", 0, 100, on_change_max)
    with cam:
        # while True:
            # frame = cam.get_frame()
            # frame_thresh = np.array(frame)
            # for x in range(frame.shape[1]):
            #     for y in range(frame.shape[0]):
            #         if not (frame[y, x] > min_val and frame[y, x] < max_val):
            #             frame_thresh[y,x] = cam.max_disparity
            # norm = cam.normalize_frame(frame_thresh)
            # color = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
            # cv2.imshow("disparity", color)

            # if cv2.waitKey(1) == ord('q'):
            #     break


        for _ in range(100):
            cam.get_frame()
        angle = 0
        while angle < 360:
            input(f"rotate object to {angle} degrees and hit enter")

            frame = cam.get_frame()
            # color = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            # cv2.imshow("disparity", color)

            scan.add_image(frame, angle)

            angle += step

        scan.save_mesh(input("enter file to save mesh (without extension): "))


if __name__ == '__main__':
    main()