#!/bin/bash
# Captures 10 frames from IMX219 on CAM0
# Uses surfaceless EGL so it works headless / via NoMachine

OUT="/home/logots/Desktop/cam_frame%d.jpg"

echo "Capturing 10 frames from CAM0 (surfaceless EGL)..."
EGL_PLATFORM=surfaceless \
gst-launch-1.0 \
    nvarguscamerasrc sensor-id=0 num-buffers=10 \
    ! 'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' \
    ! nvvidconv \
    ! jpegenc \
    ! multifilesink location="$OUT"

echo "Done. Check Desktop for cam_frame0.jpg through cam_frame9.jpg"
