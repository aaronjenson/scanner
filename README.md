# 3D Scanner

### Table of Contents

- [Introduction](#intro)
- [Video](#video)
- [Pipeline Plan](#plan)
- [Results](#results)
- [Credits](#credits)
- [Future Improvements](#improve)
- [Running The Code](#code)

## Introduction {#intro}

A little while ago, I was working on a project where I needed to get a 3D model of a person. Turns out, 3D scanners are expensive, so I spent a long time exploring photogrammetry (a process that takes a series of images of a subject from many angles and produces a 3D model) which ultimately didn't have enough accuracy for what I needed. I eventually found some software that uses an Xbox Kinect sensor as a 3D scanner, which did produce better results, but the software limited the detail of the final model on the free version of the app.  
Some time later, someone let me borrow an OAK-D device. The OAK-D is made by OpenCV and features a stereo mono camera pair, an RGB camera, and an onboard processor that can run many vision pipelines very quickly. Immediately I thought that this device should have everything needed to implement a basic 3D scanner, so I put that project idea on my list. And then I forgot about it, until it came time for this CSE 455 final project...

## Video {#video}

<a href="http://www.youtube.com/watch?feature=player_embedded&v=PQEQSz8BWYE" target="_blank"><img src="http://img.youtube.com/vi/PQEQSz8BWYE/0.jpg" alt="youtube video" width="240" height="180" border="10" /></a>

## Pipeline Plan {#plan}

General pipeline description:

1. Retrieve depth image
2. Convert depth image to point cloud
3. Crop point cloud to object
4. Transform point cloud into object's coordinate frame
5. Combine point cloud with current scan point cloud
6. Repeat 1-6 until enough data is gathered
7. Generate mesh file from point cloud

Many of these steps are pretty difficult to do without setting some constraints on the setup. Here are the assumptions I made about the scanning process:

- The camera will remain stationary throughout the scanning process
- The object will be able to be rotated to arbitrary angles through the scan[^1]
- The object's center of rotation will correspond with the center of the user-defined cropping box

These assumptions make this system a little bit less practical for an all-purpose 3D scanner, but I'll discuss in the future improvements section how some of these assumptions could be removed.

## Results {#results}

Two versions of this pipeline were created. One (`onboard_depth_cam.py`) uses the processor onboard the OAK-D device to compute depth information. The other (`offboard_depth_cam.py`) uses a stereo depth algorithm on the computer to calculate depth. 

### Offboard Depth Results

The offboard depth pipeline didn't end up working very well. The biggest problem was that processing each frame on my very underpowered laptop took up to 10 minutes per frame, and still didn't get results as accurate as the onboard version. Because of this, I primarily worked with the onboard depth pipeline for the scanner.

### Onboard Depth Results

The onboard depth computation worked surprisingly well. It could keep up with live video streams, and was pretty accurate. Any objects too close to the cameras weren't very accurate, but it would show the shapes of objects that were at an optimal distance from the device. After lots of tuning, the algorithm kind of worked to generate an stl file. The generated file really wasn't that close to the actual object, in fact it didn't look anything like the actual thing. But it did get some of the shapes right, so I think it did do something right somewhere in the process.

## Credits {#credits}

Here's the summary of what went into this project and where the ideas, algorithms, and code came from:

| Algorithm | Source |
| --- | --- |
| Depth Calculation (Onboard) | OpenCV / OAK-D |
| Depth Calculation (Offboard) | Algorithm: CSE 455, implementation: me |
| Point Cloud Operations | Open3D |
| Scanner Logic & Program Structure | me |
| Mesh Generation | Open3D |

## Future Improvements {#improve}

One of the biggest problems with producing an accurate 3D model with this method is that the depth information is very noisy. To improve our results, machine learning could be used to more accurately determine the depth at each pixel. This would also reduce the amount of filtering needed in the point cloud steps of the pipeline, as there would be less points in the wrong places.

Another possible source of improvement would make the system easier to use and more adaptable to different objects. If we assume that the object remains stationary, we can allow the cameras to be moved and we can use the resulting information to estimate the pose of the camera relative to the scan so far. This could be done using common methods from robotics, such as the SLAM algorithm that is designed to map an environment that the agent has never seen before while also maintaining accurate pose information.

## Running The Code {#code}

1. Install dependencies
  - `pip install opencv-python`
  - [depthai](https://docs.luxonis.com/projects/api/en/latest/install/)
  - `pip install open3d`
2. Connect the OAK-D device via USB
3. Run `offboard_depth_cam.py` or `onboard_depth_cam.py`
4. Follow the instructions in the console to complete the scan

[^1]: In practice, this doesn't have to be completely arbitrary. In my setup, I made marks on a rotating platform to help me rotate the object to any multiple of 15 degrees.