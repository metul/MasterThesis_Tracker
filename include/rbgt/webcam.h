// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_RBGT_WEBCAM_H_
#define OBJECT_TRACKING_INCLUDE_RBGT_WEBCAM_H_

#include <rbgt/camera.h>
#include <rbgt/common.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace rbgt {
    class Webcam : public Camera{
    public:
        ~Webcam();

        bool Init(const std::string &name);

        //void set_image_scale(float image_scale);

        // Main method
        bool UpdateImage() override;

    private:
        cv::VideoCapture device_;
        //float image_scale_ = 1.05f;
        //cv::Mat distortion_map_;
    };

}  // namespace rgbt

#endif  // OBJECT_TRACKING_INCLUDE_RBGT_WEBCAM_H_
