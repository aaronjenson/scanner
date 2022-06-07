#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import sys

class OffboardDepthCam(object):
    def __init__(self):
        self.pipeline = self.get_pipeline()
        self.device = None
        self.left_queue = None
        self.right_queue = None
        self.focal_length = 850
        self.dx = 7.5
        self.p = 0.0003 * (1280 / 640) # change last number to resolution
        self.max_disparity = self.focal_length * self.dx * self.p
        self.window_size = 5

    def __enter__(self):
        self.device = dai.Device(self.pipeline)
        self.device.__enter__()
        self.left_queue = self.device.getOutputQueue(name="left", maxSize=4, blocking=True)
        self.right_queue = self.device.getOutputQueue(name="right", maxSize=4, blocking=True)

    def __exit__(self, a, b, c):
        self.device.__exit__()
        self.device = None
        self.left_queue = None
        self.right_queue = None

    def get_pipeline(self):
        # Create pipeline
        pipeline = dai.Pipeline()

        # Define sources and outputs
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        xoutleft = pipeline.create(dai.node.XLinkOut)
        xoutright = pipeline.create(dai.node.XLinkOut)

        xoutleft.setStreamName("left")
        xoutright.setStreamName("right")

        # Properties
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)


        # Linking
        monoLeft.out.link(xoutleft.input)
        monoRight.out.link(xoutright.input)

        return pipeline
    
    def get_raw_frames(self):
        if self.device is None:
            return None
        return self.left_queue.get().getFrame(), self.right_queue.get().getFrame()

    def get_frame(self):
        left, right = self.get_raw_frames()
        if left is None or right is None:
            return None
        return self.compute_depth(left, right)

    def compute_depth(self, left, right):
        print(f"computing depth for size: {left.shape}")
        depth = np.zeros_like(left)
        right_features = [None] * (right.shape[0] * right.shape[1]) 
        for y in range(left.shape[0]):
            for x in range(left.shape[1]):
                feature = self.get_feature_descriptor(left, x, y)
                best_score = 99999
                best_x = right.shape[0]
                for rx in range(x):
                    match = self.get_feature_descriptor(right, rx, y, cache=right_features)
                    score = self.score_feature_match(feature, match)
                    if score < best_score:
                        best_score = score
                        best_x = rx
                if x == best_x:
                    depth[y, x] = self.max_disparity
                    continue
                depth[y, x] = self.focal_length * self.p * self.dx / (x - best_x)
                progress(y * left.shape[1] + x, left.shape[0] * left.shape[1])
        print()
        print("done computing depth")

        cv2.imwrite("left.png", left)
        cv2.imwrite("right.png", right)
        return depth

    def get_feature_descriptor(self, im, x, y, cache=None):
        index = x * im.shape[0] + y
        score = None
        if cache is None or cache[index] is None:
            min_x = x - self.window_size
            min_y = y - self.window_size
            max_x = x + self.window_size + 1
            max_y = y + self.window_size + 1
            min_x = max(min_x, 0)
            min_y = max(min_y, 0)
            max_x = min(max_x, im.shape[1] - 1)
            max_y = min(max_y, im.shape[0] - 1)
            score = im[min_y:max_y, min_x:max_x]
        else:
            return cache[index]
        if cache is not None:
            cache[index] = score
        return score

    def score_feature_match(self, fl, fr):
        if fl.shape != fr.shape:
            return np.Infinity
        diff = fl - fr
        sq = diff * diff
        score = np.sum(sq)
        # print(f"score: {score}")
        return score

    def get_normalized_frame(self):
        f = self.get_frame()
        if f is None:
            return None
        f = (f * (255 / self.max_disparity)).astype(np.uint8)
        return f

def progress(count, total, status=''):
    bar_len = 60
    filled_len = int(round(bar_len * count / float(total)))

    percents = round(100.0 * count / float(total), 1)
    bar = '=' * filled_len + '-' * (bar_len - filled_len)

    sys.stdout.write('[%s] %s%s ...%s\r' % (bar, percents, '%', status))
    sys.stdout.flush()

def main():
    cam = OffboardDepthCam()
    with cam:
        for _ in range(100):
            left, right = cam.get_raw_frames()
            cv2.imshow("left", left)
            cv2.imshow("right", right)
            cv2.waitKey(1)
        while True:

            # left, right = cam.get_raw_frames()
            # cv2.imshow("left", left)
            # cv2.imshow("right", right)
            frame = cam.get_normalized_frame()
            color = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            cv2.imwrite("depth.png", color)
            cv2.imshow("disparity", color)

            if cv2.waitKey(1) == ord('q'):
                break

if __name__ == '__main__':
    main()