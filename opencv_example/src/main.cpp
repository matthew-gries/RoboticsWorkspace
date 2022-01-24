#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

const int lowThreshold = 25;
const int thresholdRatio = 3;
const int kernelSize = 3;

int main(int, char**) {
    // Store captured frame here
    Mat frame, grayscaleFrame, blurFrame, detectedEdgesFrame;
    // Store the processed frame here
    Mat processedFrame;
    processedFrame.create(frame.size(), processedFrame.type());
    //--- INITIALIZE VIDEOCAPTURE
    VideoCapture cap;
    // open the default camera using default API
    // cap.open(0);
    // OR advance usage: select any API backend
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID, apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    //--- GRAB AND WRITE LOOP
    cout << "Start grabbing" << endl << "Press any key to terminate" << endl;
    for (;;) {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // Convert to grayscale
        cvtColor(frame, grayscaleFrame, COLOR_BGR2GRAY);
        // Blur
        blur(grayscaleFrame, blurFrame, Size(3, 3));
        // Canny edge detect
        Canny(blurFrame, detectedEdgesFrame, lowThreshold, lowThreshold * thresholdRatio, kernelSize);
        // check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        // show live and wait for a key with timeout long enough to show images
        imshow("Live", detectedEdgesFrame);
        if (waitKey(5) >= 0) { break; }
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}