# Object Pose Estimation Pipeline
#### For my Masters Thesis at Technical University Munich, in Collabiration with Reykjavik University
This Repository is built upon the Region-Based Gaussian Tracker by Stoiber et al.
Please find the original repository at: https://github.com/DLR-RM/3DObjectTracking/tree/master/RBGT

### Content
This repository contains our approach to a 6D tracking for mixed reality applications.
Currently, it is designed to track 2 objects and one Aruco Marker.
The main adapted source files can be found in the tracking folder.
All main functions, like starting the tracker and doing camera calibration can be found in the `tracking.cpp`.
We also adapted the camera, to better represent any webcam.
To use your own 3D models, please take a look at the `run` function in the `tracking.cpp`.
For additional information, see the usage info written by Stoiber et al.

### Including the Tracker in Unity
When including this code in Unity, we built it as a Release with Debug Information in Visual Studio for x64.
The DLLs can then be included in Unity and accessed, very similar to normal code.
If you have access to this repository, take a look at the `assets/tracking` folder in the Unity repository also developed in this thesis.
You might have to adapt from right handed to left handed coordinate system for Unity usage. we have done this in Unity, by negating the corresponding axis and mirroring the tracked object(s).

### Usage
If you want to use our tracker for your own project, we would like to refer you to `run_on_camera_sequence.cpp` for how to set up the tracker and required objects. The following variables in `run_on_camera_sequence.cpp` should be considered in an initial set up:
* `model_path`: location where all object models, that are created automatically from an .obj file, are stored.
* `body_ptr`: contains all information associated with an object
    * `geometry_path`: path to `.obj` file
    * `geometry2body_pose`: transformation that allows to set a different frame of reference for the object than defined by the `.obj` file.
    * `world2body_pose`: initial transformation between camera and object.
    * `geometry_unit_in_meter`: scale factor to scale the unit used in the `.obj` file to meter.
    * `geometry_counterclockwise`: `true` if winding order of triangles in `.obj` is defined counter-clockwise.
    * `geometry_enable_culling`: `true` if faces that are not facing toward the camera should be culled.
    * `maximum_body_diameter`: maximum diameter in meter of a sphere that encapsulates the entire body.
    * `occlusion_mask_id`: unique number between 0 and 7 that is used to encode the object in occlusion_masks.
* `model_ptr`: contains all information associated with a model
    * `sphere_radius`: distance from camera to object center
    * `n_divides`: how often an icosahedron is divided (controls the number of template views)
    * `n_points`: number of contour points

If any questions arise drop me an email at antoni@in.tum.de