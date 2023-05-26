# Face recognizer for the office manager robot

Program that does face recognition in the context of applying it to the robot assistant to greet employees of Applied Robotics company

## Installation

1. Download the archive
2. Unzip it in your ./ros2_workspace/src
3. Install requirements `pip install -r requirements.txt`
3. Build the package `colcon build --symlink-install --packages-select mngr_fr_pkg`
4. Use one of the three nodes

## Face recognition class

Class `FaceRecognizer` in the `mngr_facerec.py` file represents a number of utilities directed to to handle face detection/recognition using HOG desctiptors method and, subsequently, provides means for updating encodings of the known users.

Callable methods of the class are listed below:

- `recognize_faces` immediately performs face recognition from the videoflow. Breaks if the `encodings.pkl` file was altered of not generated at all. To generate the .pkl file use the following method.
- `update_users` updates the .pkl file accodring to the folder where the images are located.
- `get_user_id` gets the id of the user that was recognized.
- `get_json_data` loads data from .json file with which the system operates.
- `create_user` starts the gathering of images for the brand new id.

The nodes of the package use described above methods in order to perform specified tasks.

## Usage

There are three available nodes:

- `cam_pub` published the image from the web page to where the main machine translates image from its web camera. Can take two parameters `cam_src_index` and `cam_src_http` where the source of the images can be specified in one of them.
- `img_sub` subscribes to the image topic and handles detected user id - either takes images of unknown user or greets the known.
- `greet_user` outputs the message that simlutates the greeting of the recognized person.
- `upd` rewrites the `encodings.pkl` file. Use after you have added some images or updated the existing ones.

## Use case

1. Run `cam_pub` to publish images as message OR specify the different topic for passed images into `img_sub` node.
2. Run `greet` node to start listening for the name to greet.
3. Run `img_sub` node to start handling the image recognition.

After `img_sub` finds and succesfullly recognizes the user's id, it passes the name of the user to the topic `recognized_name`.
Node `greet` is subscribed to that topic so when the message is published, it does the greeting.

## Important notes

- The images used for recognition should be divided into sub-folders which are user ids to manage the labeling easier.
- Every time the sub-folders in your image folder are modified, the `upd` node must be called.
- If built without `--symlink-install` the images and .pkl file will not be found so be aware that it would need additional transporting or specifying the paths.