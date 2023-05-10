# Autonomous Stacking of Jenga Blocks

Course project for 16-662 Robot Autonomy

Team Members:
- Abhinav Gupta (`ag6`)
- Vibhakar Mohta (`vmohta`)
- Dhanvi Sreenivasan (`dsreeniv`)

Watch our YouTube demonstration:

https://youtu.be/B1N4hSssyfQ

Report:

https://drive.google.com/file/d/18JBqGNga69x8r2U_FHgkodvnfkotrfUr/view?usp=drivesdk

## Pre-requisites:
1. Install Docker
2. Install Nvidia Docker
3. Work with a system that has the franka arm configured with the [frankapy](https://github.com/iamlab-cmu/frankapy) package

## Setup:
1. Make Docker Container (use sudo if docker is not added to the user group):

   ```
   docker build -t auto_jenga .
   ```

2. Install a RealSense camera on the tool of the panda arm. To calibrate this camera follow the following steps:
    - Place an Aruco marker at a fixed location on the table
    - Run the bash script to start the docker container
        ```
        bash run_docker.sh
        ```
    -  Run the launch file which uses the [easy_handeye](https://github.com/IFL-CAMP/easy_handeye) package to calibrate the camera. Update the ID and size of the Aruco tag in the launch file:
        ```
        roslaunch block_detector realsense_easy_calib.launch
        ```

    - Record about 5-10 different values on the GUI. Make sure to rotate the arm sufficiently between different poses, and also record a few redundant poses. 
        - Use `rqt_image_view` (inside the docker container) to see if the Aruco marker is detected properly in every position. 
            ```
            bash terminal_docker.sh
            rqt_image_view
            ```
        - To freely move the robot
            ```
            bash terminal_docker.sh
            python3 guide_mode.py
            ```
            This script allows free robot movement and is configured to do the following
            - Input 1 to print joints <br>
            - Input 2 to print end effector pose <br>
            - Input 3 to exit <br>
            - Input 4 to reset joints and exit.

    - Compute the transform using the GUI. Copy and paste these values in `block_detector/config/realsense_easy.yaml`

## Downloading and configuring the dataset and models
1. Configure directory
   - `cd <frankapy package directory>`
   - `mkdir weights`

2. Download model weights
   - ```
        wget "https://drive.google.com/uc?export=download&id=18XffSSU13EzuJ_8uUEhqZJU6YAyEz4q0" -O best.pt
        ```
3. The instance segmentation dataset including Jenga blocks can be found at this [link](https://drive.google.com/file/d/1qUsEvA0-WESyihHy6-eqNq8g5hohzBKS/view?usp=share_link)
. We also provide a [tutorial](https://colab.research.google.com/drive/1GQwshAy2_xJ1y4kIV15M3OMHiS9N9oYl?usp=sharing) on how to configure and use our dataset for instance segmentation using YOLOv8. This tutorial is adapted from official [Roboflow](https://roboflow.com/) tutorial at this [link](https://colab.research.google.com/github/roboflow-ai/notebooks/blob/main/notebooks/train-yolov8-instance-segmentation-on-custom-dataset.ipynb).
 
      

## Running the Code
1. Run the start_control_pc script from the frankapy package
   - `cd <frankapy package directory>`
   - `bash ./bash_scripts/start_control_pc.sh -i [control-pc-name]`

2. Run the MoveIt Server
    - Start the docker container
      ```
      bash run_docker.sh
      ```
    - Inside the container:
        ```
        roslaunch manipulation demo_moveit.py
        ```

3. Run the block detection action server
    - Attach a terminal to the docker container
      ```
      bash terminal_docker.sh
      ```
    - Inside the container:
        ```
        roslaunch block_detector perception.launch
        ```
        This launches the realsense camera node with calibration loaded from `block_detector/config/realsense_easy.yaml`, and the block detection action server.

4. Start the stacking FSM node:
    - Attach a terminal to the docker container
      ```
      bash terminal_docker.sh
      ```
    - Inside the container:
        ```
        roslaunch fsm_handler handler.launch
        ```
        The stacking operation should begin! The motion is perpetual and the robot will build a tower and reset till the user stops the program. Happy Stacking!