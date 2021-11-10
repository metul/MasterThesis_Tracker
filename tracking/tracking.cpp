// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Edmund Stoiber, German Aerospace Center (DLR)

#include <rbgt/body.h>
#include <rbgt/common.h>
#include <rbgt/image_loader_camera.h>
#include <rbgt/normal_image_viewer.h>
#include <rbgt/occlusion_mask_renderer.h>
#include <rbgt/region_modality.h>
#include <rbgt/renderer_geometry.h>
#include <rbgt/tracker.h>
#include <rbgt/dataset_rbot_camera.h>
#include <rbgt/webcam.h>

#include <Eigen/Geometry>
#include <memory>
#include <string>

#include <thread>
#include <mutex>
#include <opencv2/aruco.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <windows.h> // For Sleep

#include "tracking.h"
namespace fs = std::filesystem;


std::shared_ptr<rbgt::objectPositions> positions;
std::shared_ptr<rbgt::marker> posMarkers;
std::shared_ptr<std::atomic<bool>> stop_tracking_;
std::thread tracking;
auto camera_ptr{std::make_shared<rbgt::Webcam>()};

//Calibration
//Get Images for calibration
//q for quitting, t for tacking pictures



fs::path getTempPath(){
    fs::path path = fs::temp_directory_path();
    path /= "6DTracking";
    if(!fs::exists(path)){
        fs::create_directory(path);
    }
    return path;
}

fs::path getAppdataPath(){
    fs::path path = fs::path(getenv("appdata"));
    path /= "6DTracking";
    if(!fs::exists(path)){
        fs::create_directory(path);
    }
    return path;
}

int takeImages() {
    int ct = 0;
    char tipka;
    char filename[100]; // For filename
    int c = 1; // For filename

    fs::path path = getTempPath();
    cv::Mat frameCalib;
    //--- INITIALIZE VIDEOCAPTURE
    cv::VideoCapture cap;
    // open the default camera using default API
    cap.open(0);
    // OR advance usage: select any API backend
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID + apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    //--- GRAB AND WRITE LOOP
    std::cout << "Start grabbing" << std::endl
              << "Press q to terminate" << std::endl;
    for (;;) {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frameCalib);

        if (frameCalib.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        Sleep(5); // Sleep is mandatory - for no leg!

        cv::putText(frameCalib, //target image
                    std::to_string(c), //text
                    cv::Point(frameCalib.cols - 50, frameCalib.rows - 40), //bottom right position
                    cv::FONT_HERSHEY_DUPLEX,
                    1.2,
                    CV_RGB(118, 185, 0), //font color
                    1.8);

        // show live and wait for a key with timeout long enough to show images
        cv::imshow("T to take Picture, Q to quit", frameCalib);  // Window name

        tipka = cv::waitKey(30);


        if (tipka == 't') {
            fs::path imgPath = path /("frame_"+std::to_string(c)+".jpg");
            cv::waitKey(10);
            cv::imshow("T to take Picture, Q to quit", frameCalib);
            cv::imwrite(imgPath.string(), frameCalib);
            std::cout << "Frame_" << c << std::endl;
            c++;
        }

        if (tipka == 'q') {
            std::cout << "Terminating..." << std::endl;
            cv::destroyAllWindows();
            Sleep(500);
            break;
        }
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{6, 9};
//Calibrate on images
int calibrate() {
    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpoints;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for (int i{0}; i < CHECKERBOARD[1]; i++) {
        for (int j{0}; j < CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j, i, 0));
    }

    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> images;

    cv::glob(getTempPath().string(), images);

    cv::Mat frame, gray;
    // vector to store the pixel coordinates of detected checker board corners
    std::vector<cv::Point2f> corner_pts;
    bool success;

    // Looping over all the images in the directory
    for (int i{0}; i < images.size(); i++) {
        frame = cv::imread(images[i]);
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        //Put text on image
        cv::putText(frame, //target image
                    "Check all pictures (Press any Key to Continue)", //text
                    cv::Point(10, 50), //top-left position
                    cv::FONT_HERSHEY_DUPLEX,
                    0.7,
                    CV_RGB(118, 185, 0), //font color
                    1.8);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts,
                                            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
                                            cv::CALIB_CB_NORMALIZE_IMAGE);

        //If desired number of corner are detected,
        //we refine the pixel coordinates and display
        //them on the images of checker board

        if (success) {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board
            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        }
        cv::imshow("Calibration", frame);

        cv::waitKey(0);
    }
    //DESTROY EVERYTHING
    cv::destroyAllWindows();
    fs::remove_all(getTempPath());

    cv::Mat cameraMatrix, distCoeffs, R, T;


    //Performing camera calibration by
    //passing the value of known 3D points (objpoints)
    //and corresponding pixel coordinates of the
    //detected corners (imgpoints)

    cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);

    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl;

    rbgt::Intrinsics intrinsics;

    intrinsics.fu=cameraMatrix.at<double>(0,0);
    intrinsics.fv=cameraMatrix.at<double>(1,1);
    intrinsics.ppu=cameraMatrix.at<double>(0,2);
    intrinsics.ppv=cameraMatrix.at<double>(1,2);
    intrinsics.width = frame.cols;
    intrinsics.height = frame.rows;

    rbgt::DistCoeffs distc;
    distc.k1 = distCoeffs.at<double>(0);
    distc.k2 = distCoeffs.at<double>(1);
    distc.p1 = distCoeffs.at<double>(2);
    distc.p2 = distCoeffs.at<double>(3);
    distc.k3 = distCoeffs.at<double>(4);

    std::ofstream distFile;
    distFile.open(getAppdataPath()/"distCoeffs.txt");
    rbgt::WriteValueToFile(distFile, "distCoeffs_", distc);
    distFile.close();

    std::ofstream intrinsicsFile;
    intrinsicsFile.open(getAppdataPath()/"intrinsics.txt");
    rbgt::WriteValueToFile(intrinsicsFile, "intrinsics_", intrinsics);
    intrinsicsFile.close();
    return 0;
}


//Stuff for Tracking

rbgt::objects getData() {
    std::unique_lock _lock(positions->m);
    return positions->objects123;
}

rbgt::markerPos getMarker() {
    std::unique_lock _lock2(posMarkers->n);
    return posMarkers->markerVecs;
}

void init() {
    auto object_positions{std::make_shared<rbgt::objectPositions>()};
    auto marker_{std::make_shared<rbgt::marker>()};
    stop_tracking_ = std::make_shared<std::atomic<bool>>(false);
    positions = object_positions;
    posMarkers = marker_;
}

void startTracking() {
    std::cout << "start" << std::endl;
    init();
    std::cout << "init finished" << std::endl;
    tracking = std::thread(run);
    // Testing only:
    // tracking.join();
}

void endTracking() {
    //atomic bool for exiting tracking loop
    *stop_tracking_ = true;
    std::cout << "Join starting" << std::endl;
    tracking.join();
    std::cout << "Join finished" << std::endl;
}

int run() {

    // Change accordingly
    const std::string sequence_path{"C:\\Projects\\Valentin\\Data"};
    const std::string model_path{"C:\\Projects\\Valentin\\Data\\temp"};

    constexpr bool kSaveViewerImage = false;
    const std::string viewer_save_path{"C:\\Projects\\Valentin\\Data\\temp\\frames"};

    // Set up tracker and renderer geometry
    auto tracker_ptr{std::make_shared<rbgt::Tracker>(positions, posMarkers)};
    auto renderer_geometry_ptr{std::make_shared<rbgt::RendererGeometry>()};

    // Set up camera
    //tracking with RBOT Dataset
    //auto camera_ptr{std::make_shared<rbgt::DatasetRBOTCamera>()};
    //camera_ptr->Init("camera", sequence_path, "cam", "a_regular", 0);
    //tracking live feed
    auto camera_ptr{std::make_shared<rbgt::Webcam>()};
    camera_ptr->Init("camera");

    // Set up viewers
    auto viewer_ptr{std::make_shared<rbgt::NormalImageViewer>()};
    viewer_ptr->Init("viewer", renderer_geometry_ptr, camera_ptr);
    if (kSaveViewerImage) viewer_ptr->StartSavingImages(viewer_save_path);
    tracker_ptr->AddViewer(viewer_ptr);

    // Set up body1 (change accordingly)
    const std::string body1_geometry_path{"C:\\Projects\\Valentin\\Data\\bunny\\bunny.obj"};
    rbgt::Transform3fA body1_geometry2body_pose{Eigen::Translation3f(0.0f, 0.0f, 0.0f)};
    rbgt::Transform3fA body1_world2body_pose;
    body1_world2body_pose.matrix() <<-0.0353159,-0.350179, 0.913716,-0.413982,
    -0.0673792,  -0.914628,  -0.407659,   0.310014,
    0.98925, -0.0343645,   0.072784,  0.0956292,0,0,0,1;


    //0.00056 for Hannes
    //0.0008
    auto body1_ptr{std::make_shared<rbgt::Body>("body1", body1_geometry_path,
                                                0.0008f, true, false, 0.60f,
                                                body1_geometry2body_pose)};
    body1_ptr->set_world2body_pose(body1_world2body_pose);
    body1_ptr->set_occlusion_mask_id(1);
    renderer_geometry_ptr->AddBody(body1_ptr);

    // Set up model body 1
    const std::string body1_model_name{"bunny"};
    auto body1_model_ptr{std::make_shared<rbgt::Model>(body1_model_name)};
    if (!body1_model_ptr->LoadModel(model_path, body1_model_name)) {
        body1_model_ptr->GenerateModel(*body1_ptr, 0.8f, 4, 200);
        body1_model_ptr->SaveModel(model_path, body1_model_name);
    }

    // Set up region modality body 1
    auto body1_region_modality_ptr{std::make_shared<rbgt::RegionModality>()};
    body1_region_modality_ptr->Init("body1_region_modality", body1_ptr,
                                    body1_model_ptr, camera_ptr);
    tracker_ptr->AddRegionModality(body1_region_modality_ptr);


    // Set up body2 (change accordingly), if needed add more objects
    /*
    const std::string body2_geometry_path{"C:\\Projects\\Valentin\\Data\\moai_scale\\moai_scale.obj"};
    rbgt::Transform3fA body2_geometry2body_pose{Eigen::Translation3f(0.0f, 0.0f, 0.0f)};
    rbgt::Transform3fA body2_world2body_pose;
    body2_world2body_pose.matrix() <<  -0.0503092, -0.403222, 0.887141, -0.456749,
    -0.00502005, -0.893027, -0.455081, 0.363051,
    0.990884, 0.0153704, 0.110556, -0.182848,0,0,0,1;

    auto body2_ptr{std::make_shared<rbgt::Body>("moai", body2_geometry_path,
                                                0.001f, true, false, 0.60f,
                                                body2_geometry2body_pose)};
    body2_ptr->set_world2body_pose(body2_world2body_pose);
    body2_ptr->set_occlusion_mask_id(2);
    renderer_geometry_ptr->AddBody(body2_ptr);

    // Set up model body 2
    const std::string body2_model_name{"moai"};
    auto body2_model_ptr{std::make_shared<rbgt::Model>(body2_model_name)};
    if (!body2_model_ptr->LoadModel(model_path, body2_model_name)) {
        body2_model_ptr->GenerateModel(*body2_ptr, 0.8f, 4, 200);
        body2_model_ptr->SaveModel(model_path, body2_model_name);
    }

    // Set up region modality body 2
    auto body2_region_modality_ptr{std::make_shared<rbgt::RegionModality>()};
    body2_region_modality_ptr->Init("body2_region_modality", body2_ptr,
                                    body2_model_ptr, camera_ptr);
    tracker_ptr->AddRegionModality(body2_region_modality_ptr);
    */


    // Set up occlusion mask renderer
    auto occlusion_mask_renderer_ptr{
            std::make_shared<rbgt::OcclusionMaskRenderer>()};
    occlusion_mask_renderer_ptr->InitFromCamera(
            "occlusion_mask_renderer", renderer_geometry_ptr, *camera_ptr);

    body1_region_modality_ptr->UseOcclusionHandling(occlusion_mask_renderer_ptr);
    //body2_region_modality_ptr->UseOcclusionHandling(occlusion_mask_renderer_ptr);

    // Start tracking
    tracker_ptr->StartTracker(false, stop_tracking_);
    return 0;
}
