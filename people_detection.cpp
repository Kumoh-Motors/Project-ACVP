#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <chrono>
#include <thread>

using namespace cv;
using namespace cv::dnn;
using namespace std;

struct SharedData {
    bool person_detected;
    bool program_running;
};

// Structure to store detection results of the previous frame
struct DetectionResult {
    vector<Rect> boxes;
    vector<float> confidences;
    bool valid;
    chrono::steady_clock::time_point timestamp;
};

int main() {
    // Shared memory setup
    key_t key = ftok("/tmp", 65);
    int shmid = shmget(key, sizeof(SharedData), 0666|IPC_CREAT);
    if (shmid == -1) {
        cerr << "Failed to create shared memory" << endl;
        return 1;
    }

    SharedData* sharedData = (SharedData*)shmat(shmid, NULL, 0);
    if (sharedData == (void*)-1) {
        cerr << "Failed to connect to shared memory" << endl;
        return 1;
    }

    sharedData->program_running = true;
    sharedData->person_detected = false;

    // YOLO model setup
    string modelConfiguration = "yolov3-tiny.cfg";
    string modelWeights = "yolov3-tiny.weights";
    
    Net net = readNetFromDarknet(modelConfiguration, modelWeights);
    if (net.empty()) {
        cerr << "Unable to load the model!" << endl;
        return -1;
    }

    net.setPreferableBackend(DNN_BACKEND_CUDA);
    net.setPreferableTarget(DNN_TARGET_CUDA);

    string pipeline = "libcamerasrc ! video/x-raw, width=320, height=320, framerate=30/1 ! "
                     "videoconvert ! video/x-raw, format=BGR ! "
                     "queue max-size-buffers=2 leaky=2 ! "
                     "appsink drop=true max-buffers=2";
    VideoCapture cap(pipeline, CAP_GSTREAMER);
    
    cap.set(CAP_PROP_BUFFERSIZE, 2);

    vector<String> outLayerNames = net.getUnconnectedOutLayersNames();
    float confThreshold = 0.6;
    float nmsThreshold = 0.45;

    Mat frame;
    int skip_frames = 0;
    const int PROCESS_EVERY_N_FRAMES = 4;
    const int DETECTION_VALIDITY_MS = 500;  // Detection result validity time (milliseconds)

    // Variable to store previous detection results
    DetectionResult prevResult = {vector<Rect>(), vector<float>(), false, chrono::steady_clock::now()};

    while (sharedData->program_running) {
        cap >> frame;
        if (frame.empty()) continue;

        auto currentTime = chrono::steady_clock::now();
        bool needNewDetection = false;

        // Validity check of previous results
        if (!prevResult.valid || 
            chrono::duration_cast<chrono::milliseconds>(currentTime - prevResult.timestamp).count() > DETECTION_VALIDITY_MS) {
            needNewDetection = true;
        }

        skip_frames = (skip_frames + 1) % PROCESS_EVERY_N_FRAMES;
        if (skip_frames != 0 && !needNewDetection) {
            // Display previous detection results
            if (prevResult.valid) {
                for (size_t i = 0; i < prevResult.boxes.size(); i++) {
                    rectangle(frame, prevResult.boxes[i], Scalar(0, 255, 0), 2);
                    string label = format("Person: %.2f", prevResult.confidences[i]);
                    putText(frame, label, Point(prevResult.boxes[i].x, prevResult.boxes[i].y - 5),
                            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
                }
                sharedData->person_detected = true;
            } else {
                sharedData->person_detected = false;
            }
            
            imshow("Person Detection", frame);
            if (waitKey(1) == 27) break;
            continue;
        }

        // Perform new detection
        Mat blob;
        blobFromImage(frame, blob, 1/255.0, Size(320, 320), Scalar(), true, false, CV_32F);
        net.setInput(blob);

        vector<Mat> outs;
        net.forward(outs, outLayerNames);

        vector<Rect> boxes;
        vector<float> confidences;
        bool person_found = false;

        // Process detection results
        for (const auto& out : outs) {
            float* data = (float*)out.data;
            for (int j = 0; j < out.rows; ++j, data += out.cols) {
                if (data[4] < confThreshold) continue;

                float* classes_scores = data + 5;
                if (classes_scores[0] > confThreshold) {
                    person_found = true;
                    
                    int centerX = (int)(data[0] * frame.cols);
                    int centerY = (int)(data[1] * frame.rows);
                    int width = (int)(data[2] * frame.cols);
                    int height = (int)(data[3] * frame.rows);
                    
                    boxes.push_back(Rect(centerX - width/2, centerY - height/2, width, height));
                    confidences.push_back(classes_scores[0]);
                }
            }
        }

        vector<int> indices;
        if (!boxes.empty()) {
            NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
            
            prevResult.boxes.clear();
            prevResult.confidences.clear();
            
            for (int idx : indices) {
                prevResult.boxes.push_back(boxes[idx]);
                prevResult.confidences.push_back(confidences[idx]);
                
                rectangle(frame, boxes[idx], Scalar(0, 255, 0), 2);
                string label = format("Person: %.2f", confidences[idx]);
                putText(frame, label, Point(boxes[idx].x, boxes[idx].y - 5),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
            }
            
            prevResult.valid = true;
            prevResult.timestamp = currentTime;
        } else {
            prevResult.valid = false;
        }

        sharedData->person_detected = person_found;

        imshow("Person Detection", frame);
        if (waitKey(1) == 27) break;
    }

    sharedData->program_running = false;
    shmdt(sharedData);
    shmctl(shmid, IPC_RMID, NULL);
    
    return 0;
}
