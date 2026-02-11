# DWM1001 ROS2

This repository tracks the documentation and source for the ROS2 driver and support packages related to the [Qorvo DWM1001](https://www.qorvo.com/products/p/DWM1001-DEV) UWB sensor.
The HIVE Lab uses these sensors as a reasonable, low cost alternative to a full motion capture system for locating robots in a space.

## ros bag play 
ros2 bag play /perception_dataset/2026_02_11_convoy/bag_file_2026_02_08_test1_ben_shemen/bag_file_2026_02_08_10_14_59_0.mcap -l -r 0.5  --clock-topics-all

## Dependencies

This package requires the installation of our [`pydwm1001`](https://github.com/the-hive-lab/pydwm1001) library.

## Packages

- [`dwm1001_driver`](dwm1001_driver/README.md): The core ROS2 driver.
- [`dwm1001_launch`](dwm1001_launch/README.md): The launch configurations for active and passive node.
- [`dwm1001_transform`](dwm1001_transform/README.md): A package that provides transformations from the `dwm1001` frame to the `map` frame.
- [`dwm1001_visualization`](dwm1001_visualization/README.md): This package provides visualizations in `rviz` for a DWM1001 deployment.

# Find all relevant ports using 
ls -la /dev/ttyACM* 2>/dev/null; echo "---"; dmesg | grep -i "ttyACM\|cdc_acm"
 
 # How to find if it's publish or not 
 /dwm1001/dwm1001/uwb_ranges
ros2 topic hz /dwm1001/dwm1001/uwb_ranges 
ros2 topic echo /dwm1001/dwm1001/uwb_ranges  --once 

ros2 topic hz /dwm10011_left/dwm10011_left/tag_left/uwb_ranges
ros2 topic hz /dwm10011_left/dwm10011_left/tag_right/uwb_ranges
ros2 topic echo /dwm10011_left/dwm10011_left/tag_left/uwb_ranges --once 

## How to use this repositroy under algo / ros2 workspace 
how to add and compile this code for deveoper purposes 


    # Remove paths, handling symlinks properly (remove symlink itself, not the target file)
    <!-- for p in /ros2_ws/src/dwm1001_ros2 /ros2_ws/src/dwm1001_driver src/dwm1001_ros2/dwm1001_driver /ros2_ws/install/dwm1001_driver/ /ros2_ws/build/dwm1001_driver/ /ros2_ws/local/dwm1001_driver/; do [ -L "$p" ] && rm "$p" || rm -rf "$p" 2>/dev/null; done -->

    ln -s /perception_code/ros_recording_system/src/dwm1001_ros2/ /ros2_ws/src/ && 
    
    cd /ros2_ws && colcon build --symlink-install --packages-select dwm1001_driver dwm1001_msg && source /ros2_ws/install/setup.bash  
    pip list | grep dwm1001

    ros2 pkg list | grep dwm1001_driver 
    ros2 pkg executables dwm1001_driver
    # print all executables for each package 
    for pkg in $(ros2 pkg list | grep -E 'dwm1001_driver'); do
        echo "=== Executables for $pkg ==="
        ros2 pkg executables $pkg
        echo
    done

