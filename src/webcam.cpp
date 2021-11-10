// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include <rbgt/webcam.h>
namespace fs = std::filesystem;

namespace rbgt {
    fs::path getAppdataPath(){
        fs::path path = fs::path(getenv("appdata"));
        path /= "6DTracking";
        if(!fs::exists(path)){
            fs::create_directory(path);
        }
        return path;
    }

    Webcam::~Webcam() {
        if (initialized_) {
            device_.release();
        }
    }

    bool Webcam::Init(const std::string &name) {
        name_ = name;

        if (!initialized_) {
            if (!device_.open(0)) {
                return false;
            }
            // Load multiple images to adjust to white balance
            constexpr int kNumberImagesDropped = 10;
            for (int i = 0; i < kNumberImagesDropped; ++i) {
                while (!device_.grab());
            }
            /*
          constexpr int kTimeoutInMs = 100;
          constexpr int kNumberImagesDropped = 10;

          // Configure color camera
          config_ = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
          config_.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
          config_.color_resolution = K4A_COLOR_RESOLUTION_720P;

          // Check if camera is available
          if (k4a::device::get_installed_count() == 0) return false;

          // Start camera
          device_ = k4a::device::open(K4A_DEVICE_DEFAULT);
          device_.start_cameras(&config_);

          // Load multiple images to adjust to white balance
          for (int i = 0; i < kNumberImagesDropped; ++i) {
            while (!device_.get_capture(&capture_,
                                        std::chrono::milliseconds{kTimeoutInMs}))
              ;
          }
             */
        }
        // Load intrinsics from camera
        /*
        const k4a_calibration_camera_t calibration{
            device_.get_calibration(config_.depth_mode, config_.color_resolution)
                .color_camera_calibration};
        const k4a_calibration_intrinsic_parameters_t::_param param =
            calibration.intrinsics.parameters.param;
            */
        //K-Matrix
        //float fx = 478.671;
        //float fy = 478.305;
        //float cx = 316.813;
        //float cy = 242.758;
        Intrinsics intrinsics;
        std::ifstream intrinsicsFile;
        intrinsicsFile.open(getAppdataPath()/"intrinsics.txt", std::ios::binary);
        rbgt::ReadValueFromFile(intrinsicsFile, &intrinsics);
        intrinsicsFile.close();

        intrinsics_.fu = intrinsics.fu;
        intrinsics_.fv = intrinsics.fv;
        intrinsics_.ppu = intrinsics.ppu;
        intrinsics_.ppv = intrinsics.ppv;
        intrinsics_.width = device_.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_WIDTH);
        intrinsics_.height = device_.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT);

        DistCoeffs distc;
        std::ifstream distFile;
        distFile.open(getAppdataPath()/"distCoeffs.txt", std::ios::binary);
        rbgt::ReadValueFromFile(distFile, &distc);
        distFile.close();

        distcoeffs_.k1 = distc.k1;
        distcoeffs_.k2 = distc.k2;
        distcoeffs_.p1 = distc.p1;
        distcoeffs_.p2 = distc.p2;
        distcoeffs_.k3 = distc.k3;

        // Scale intrinsics acording to image scale
        //intrinsics_.fu *= image_scale_;
        //intrinsics_.fv *= image_scale_;

        // Calculate distortion map
        /*
        cv::Mat1f camera_matrix(3, 3);
        camera_matrix << fx, 0, cx, 0, fy, cy, 0, 0, 1;
        cv::Mat1f new_camera_matrix(3, 3);
        new_camera_matrix << intrinsics_.fu, 0, intrinsics_.ppu, 0, intrinsics_.fv,
                intrinsics_.ppv, 0, 0, 1;

        cv::Mat1f distortion_coeff(1, 8);
        distortion_coeff << param.k1, param.k2, param.p1, param.p2, param.k3, param.k4, param.k5, param.k6;
        cv::Mat map1, map2, map3;
        cv::initUndistortRectifyMap(
                camera_matrix, distortion_coeff, cv::Mat{}, new_camera_matrix,
                cv::Size{intrinsics_.width, intrinsics_.height}, CV_32FC1, map1, map2);
        cv::convertMaps(map1, map2, distortion_map_, map3, CV_16SC2, true);
        */
        // Update image
        UpdateImage();
        initialized_ = true;
        return true;
    }

    bool Webcam::UpdateImage() {
        // Get image
        device_ >> image_;
        // Undistort image
        /*
        cv::Mat temp_image;
        cv::cvtColor(cv::Mat{cv::Size{intrinsics_.width, intrinsics_.height}, CV_8UC4,
                             (void *) k4a_image.get_buffer(), cv::Mat::AUTO_STEP},
                     temp_image, cv::COLOR_RGBA2RGB);
        cv::remap(temp_image, image_, distortion_map_, cv::Mat(), cv::INTER_NEAREST,
                  cv::BORDER_CONSTANT);
        */
        SaveImageIfDesired();
        return true;
    }

}  // namespace rbgt
