// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include <rbgt/tracker.h>
#include <mutex>
#include <Eigen/Dense>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>

namespace rbgt {

    void Tracker::AddRegionModality(
            std::shared_ptr<RegionModality> region_modality_ptr) {
        region_modality_ptrs_.push_back(std::move(region_modality_ptr));
    }

    void Tracker::AddViewer(std::shared_ptr<Viewer> viewer_ptr) {
        viewer_ptrs_.push_back(std::move(viewer_ptr));
    }

    void Tracker::set_n_corr_iterations(int n_corr_iterations) {
        n_corr_iterations_ = n_corr_iterations;
    }

    void Tracker::set_n_update_iterations(int n_update_iterations) {
        n_update_iterations_ = n_update_iterations;
    }

    void Tracker::set_visualization_time(int visualization_time) {
        visualization_time_ = visualization_time;
    }

    void Tracker::set_viewer_time(int viewer_time) { viewer_time_ = viewer_time; }

    void Tracker::StartTracker(bool start_tracking, std::shared_ptr<std::atomic<bool>> stop_tracking_) {
        start_tracking_ = start_tracking;
        SetUpObjects();
        cv::Vec3d rvecObj0, rvecObj1;
        //For Correcting the coordinate system
        cv::Matx33d correctionMat (0,1,0,0,0,1,1,0,0);
        cv::Matx33d rotY (0,0,1,0,1,0,-1,0,0);
        cv::Matx33d rotX (-1,0,0,0,1,0,0,0,1);

        //setup camera calibration for aruco

        DistCoeffs distc = camera_ptrs_[0]->Camera::distcoeffs();
        cv::Mat distCoeffs = (cv::Mat_<float>(1, 5) << distc.k1, distc.k2, distc.p1, distc.p2, distc.k3);
        Intrinsics intr = camera_ptrs_[0]->Camera::intrinsics();
        cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << intr.fu, 0, intr.ppu, 0, intr.fv, intr.ppv, 0, 0, 1);

        //setup Aruco
        cv::Mat markerImage;
        cv::Mat inputImage = camera_ptrs_[0]->image();
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);

        for (int iteration = 0; !*stop_tracking_; ++iteration) {
            //object tracking
            if (start_tracking_) {
                if (!StartRegionModalities()) return;
                tracking_started_ = true;
                start_tracking_ = false;
            }
            if (tracking_started_) {
                if (!ExecuteTrackingCycle(iteration)) return;
            } else {
                if (!ExecuteViewingCycle(iteration)) return;
            }

            //aruco marker tracking
            cv::Mat image, imageCopy;
            image = camera_ptrs_[0]->image();
            image.copyTo(imageCopy);
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f> > corners;
            cv::aruco::detectMarkers(image, dictionary, corners, ids);
            if (ids.size() > 0) cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            std::vector<cv::Mat> markerPositions;
            //SWITCH
            cv::aruco::estimatePoseSingleMarkers(corners, 0.10, cameraMatrix, distCoeffs, rvecs, tvecs);
            for (int i = 0; i < ids.size(); i++) {
                cv::Matx33d markerM;
                cv::Rodrigues(rvecs[i], markerM);
                markerM = markerM * correctionMat.inv();
                cv::Rodrigues(markerM, rvecs[i]);
                cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

                std::unique_lock lock_(marker_->n);
                marker_->markerVecs.tvec.pos0 = tvecs[i][0];
                marker_->markerVecs.tvec.pos1 = tvecs[i][1];
                marker_->markerVecs.tvec.pos2 = tvecs[i][2];
                marker_->markerVecs.rvec.pos0 = rvecs[i][0];
                marker_->markerVecs.rvec.pos1 = rvecs[i][1];
                marker_->markerVecs.rvec.pos2 = rvecs[i][2];
            }
            //safe position of objects
                auto pos = region_modality_ptrs_[0]->body_ptr()->body2world_pose();
                //Matrix to rvec
                auto rotation = cv::Matx33d(pos(0, 0), pos(0, 1), pos(0, 2),
                                            pos(1, 0), pos(1, 1), pos(1, 2),
                                            pos(2, 0), pos(2, 1), pos(2, 2));
                //rotation = rotation * correctionMat;
                cv::Rodrigues(rotation, rvecObj0);
                //Matrix to tvec
                auto tvecObj0 = cv::Vec3d(pos(0, 3), pos(1, 3), pos(2, 3));

            {
                std::unique_lock _lock(object_positions_->m);
                //assign values to ptr
                object_positions_->objects123.object0.rvec.pos0 = rvecObj0[0];
                object_positions_->objects123.object0.rvec.pos1 = rvecObj0[1];
                object_positions_->objects123.object0.rvec.pos2 = rvecObj0[2];
                object_positions_->objects123.object0.tvec.pos0 = tvecObj0[0];
                object_positions_->objects123.object0.tvec.pos1 = tvecObj0[1];
                object_positions_->objects123.object0.tvec.pos2 = tvecObj0[2];
            }
                //For finding correct axis, delete for later use:
                cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecObj0, tvecObj0, 0.1);

            //object 2, if needed add more objects
            /*
            pos = region_modality_ptrs_[1]->body_ptr()->body2world_pose();
            //Matrix to rvec
            rotation = cv::Matx33d(pos(0, 0), pos(0, 1), pos(0, 2),
                                        pos(1, 0), pos(1, 1), pos(1, 2),
                                        pos(2, 0), pos(2, 1), pos(2, 2));
            //rotation = rotation * correctionMat;

            cv::Rodrigues(rotation, rvecObj1);
            //Matrix to tvec
            auto tvecObj1 = cv::Vec3d(pos(0, 3), pos(1, 3), pos(2, 3));

            {
                std::unique_lock _lock(object_positions_->m);
                //assign values to ptr
                object_positions_->objects123.object1.rvec.pos0 = rvecObj1[0];
                object_positions_->objects123.object1.rvec.pos1 = rvecObj1[1];
                object_positions_->objects123.object1.rvec.pos2 = rvecObj1[2];
                object_positions_->objects123.object1.tvec.pos0 = tvecObj1[0];
                object_positions_->objects123.object1.tvec.pos1 = tvecObj1[1];
                object_positions_->objects123.object1.tvec.pos2 = tvecObj1[2];
            }

            cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecObj1, tvecObj1,0.1);
            */
            cv::imshow("marker", imageCopy);
        }
        //close all OpenCV windows, necessary for Unity
        cv::destroyAllWindows();
    }

    void Tracker::SetUpObjects() {
        camera_ptrs_.clear();
        occlusion_mask_renderer_ptrs_.clear();
        for (auto &region_modality_ptr : region_modality_ptrs_) {
            if (region_modality_ptr->camera_ptr())
                AddPtrIfNameNotExists(region_modality_ptr->camera_ptr(), &camera_ptrs_);
            if (region_modality_ptr->occlusion_mask_renderer_ptr())
                AddPtrIfNameNotExists(region_modality_ptr->occlusion_mask_renderer_ptr(),
                                      &occlusion_mask_renderer_ptrs_);
        }
        for (auto &viewer_ptr : viewer_ptrs_) {
            if (viewer_ptr->camera_ptr())
                AddPtrIfNameNotExists(viewer_ptr->camera_ptr(), &camera_ptrs_);
        }
    }

    bool Tracker::ExecuteViewingCycle(int iteration) {
        if (!UpdateCameras()) return false;
        return UpdateViewers(iteration);
    }

    bool Tracker::ExecuteTrackingCycle(int iteration) {
        if (!CalculateBeforeCameraUpdate()) return false;
        if (!UpdateCameras()) return false;
        for (int corr_iteration = 0; corr_iteration < n_corr_iterations_;
             ++corr_iteration) {
            int corr_save_idx = iteration * n_corr_iterations_ + corr_iteration;
            if (!StartOcclusionMaskRendering()) return false;
            if (!CalculateCorrespondences(corr_iteration)) return false;
            if (!VisualizeCorrespondences(corr_save_idx)) return false;
            for (int update_iteration = 0; update_iteration < n_update_iterations_;
                 ++update_iteration) {
                int update_save_idx =
                        corr_save_idx * n_update_iterations_ + update_iteration;
                if (!CalculatePoseUpdate()) return false;
                if (!VisualizePoseUpdate(update_save_idx)) return false;
            }
        }
        if (!VisualizeResults(iteration)) return false;
        if (!UpdateViewers(iteration)) return false;
        return true;
    }

    bool Tracker::StartRegionModalities() {
        for (auto &region_modality_ptr : region_modality_ptrs_) {
            if (!region_modality_ptr->StartModality()) return false;
        }
        return true;
    }

    bool Tracker::CalculateBeforeCameraUpdate() {
        for (auto &region_modality_ptr : region_modality_ptrs_) {
            if (!region_modality_ptr->CalculateBeforeCameraUpdate()) return false;
        }
        return true;
    }

    bool Tracker::UpdateCameras() {
        for (auto &camera_ptr : camera_ptrs_) {
            if (!camera_ptr->UpdateImage()) return false;
        }
        return true;
    }

    bool Tracker::StartOcclusionMaskRendering() {
        for (auto &occlusion_renderer_ptr : occlusion_mask_renderer_ptrs_) {
            if (!occlusion_renderer_ptr->StartRendering()) return false;
        }
        return true;
    }

    bool Tracker::CalculateCorrespondences(int corr_iteration) {
        for (auto &region_modality_ptr : region_modality_ptrs_) {
            if (!region_modality_ptr->CalculateCorrespondences(corr_iteration))
                return false;
        }
        return true;
    }

    bool Tracker::VisualizeCorrespondences(int save_idx) {
        bool imshow_correspondences = false;
        for (auto &region_modality_ptr : region_modality_ptrs_) {
            if (!region_modality_ptr->VisualizeCorrespondences(save_idx)) return false;
            if (region_modality_ptr->imshow_correspondence())
                imshow_correspondences = true;
        }
        if (imshow_correspondences) {
            if (cv::waitKey(visualization_time_) == 'q') return false;
        }
        return true;
    }

    bool Tracker::CalculatePoseUpdate() {
        for (auto &region_modality_ptr : region_modality_ptrs_) {
            if (!region_modality_ptr->CalculatePoseUpdate()) return false;
        }
        return true;
    }

    bool Tracker::VisualizePoseUpdate(int save_idx) {
        bool imshow_pose_update = false;
        for (auto &region_modality_ptr : region_modality_ptrs_) {
            if (!region_modality_ptr->VisualizePoseUpdate(save_idx)) return false;
            if (region_modality_ptr->imshow_pose_update()) imshow_pose_update = true;
        }
        if (imshow_pose_update) {
            if (cv::waitKey(visualization_time_) == 'q') return false;
        }
        return true;
    }

    bool Tracker::VisualizeResults(int save_idx) {
        bool imshow_result = false;
        for (auto &region_modality_ptr : region_modality_ptrs_) {
            if (!region_modality_ptr->VisualizeResults(save_idx)) return false;
            if (region_modality_ptr->imshow_result()) imshow_result = true;
        }
        if (imshow_result) {
            if (cv::waitKey(visualization_time_) == 'q') return false;
        }
        return true;
    }

    bool Tracker::UpdateViewers(int iteration) {
        if (!viewer_ptrs_.empty()) {
            for (auto &viewer_ptr : viewer_ptrs_) {
                viewer_ptr->UpdateViewer(iteration);
            }
            char key = cv::waitKey(viewer_time_);
            if (key == 't' && !tracking_started_)
                start_tracking_ = true;
            else if (key == 'q')
                return false;
            else if (key == 's')
                std::cout<<"jou"<<std::endl;
        }
        return true;
    }

    std::vector<std::shared_ptr<RegionModality>> Tracker::region_modality_ptrs()
    const {
        return region_modality_ptrs_;
    }

    std::vector<std::shared_ptr<Viewer>> Tracker::viewer_ptrs() const {
        return viewer_ptrs_;
    }

    int Tracker::n_corr_iterations() const { return n_corr_iterations_; }

    int Tracker::n_update_iterations() const { return n_update_iterations_; }

    int Tracker::visualization_time() const { return visualization_time_; }

    int Tracker::viewer_time() const { return viewer_time_; }

}  // namespace rbgt
