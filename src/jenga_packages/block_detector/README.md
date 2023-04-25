How to calibrate camera
Launch the Azure Kinnect Camera in the PC
```
roslaunch azure_kinect_ros_driver driver.launch
```

- Start the GUI. In the docker terminal 
    ```
    roslaunch block_detector easy_calib.launch 
    ``` 
    This will launch a GUI window where you can take multiple samples. It also launches the aruco node. Please configure as required in the launch file for the tf frames and aruco marker ids. The easy_calibration package from github is used and it reads from the tf tree of the robot.

- Move the robot to multiple positions and take multiple readings. Use the rqt_image_viewer on the PC to see if the aruco marker is detected properly at a position. To run the run_guide_mode, use the following command.
   
    In docker terminal
    ```
    cd /home/ros_ws/src/jenga_packages
    python3 run_guide_mode.py 
    ```

- Once you have multiple readings (around 5), click the compute button in the GUI. This will print the calibration values. Copy and paste them at `src/jenga_packages/block_detector/config`

- The node to use this transform is written can can be launched as
    ```
    rosrun block_detector easy_tf_publisher.py 
    ```
    This node has already been added in the perception.launch file and reads params from azure_easy.yaml.