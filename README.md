# Face recognizer for the office manager robot

Program that does face recognition in the context of applying it to the robot assistant to greet employees of Applied Robotics company

## Installation

1. Install the mngr_facrec package from https://github.com/Draskown/facerec_lib
2. Download the archive
3. Unzip it in your ./ros2_workspace/src
4. Build the package `colcon build --symlink-install --packages-select mngr_fr_pkg`
5. Use one of the three nodes.

## Usage

There are three available nodes:

- `cam_pub` published the image from the web page to where the main machine translates image from its web camera.
- `img_sub` subscribes to the image topic and handles detected user id - either takes images of unknown user or greets the known
- `upd` rewrites the `encodings.pkl` file. Use after you have added some images or updated the existing ones.

## Important notes

- The images used for recognition should be divided into sub-folders which are user ids to manage the labeling easier.
- Every time the sub-folders in your image folder are modified, the `upd` node must be called.
- If built without `--symlink-install` the images and .pkl file will not be found so be aware that it would need additional transporting or specifying the paths.