# fsg_package overview

![alt text](rosgraph.png)

This package has two nodes. The main node (fsg) reads in images, processses the images using FSG and publishes the resulting images as well as the segments. The idea was that you do most of the processing in C++ before handing over results to python. However, the segments are so noisy that (given the time) I'd probably do more filtering in C++ before publishing the final results to python. One good thing about this architecture is that it is modular and so the improved node with more filtering might be a drop in replacement for the current one.

The other node in this package (test_input) provides the video for the main node (fsg) via ROS image_transport, which uses a smart pointer for image sharing when the publish and subscribe are from the same machine.
