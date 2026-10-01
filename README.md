# final_v1
\<code for sea trial, no sim\> <br>
the arm requires point cloud and grasp pose data from the dwe driver and axis_grasp package.

## general workflow:
1. sync code <br>
2. ssh into vehicle <br>
3. build this package <br>
4. launch dwe driver <br>
5. launch axis_grasp <br>
6. launch this mission <br>
7. send command to arm <br>
8. arm attempts to grab handle <br>

## commands:
1. sync code
```bash
rsync -av --delete src/final_v1/ aib@192.168.1.1:arm_ws/src/final_v1/
```
2. ssh into vehicle
```bash
ssh aib@192.168.1.1
```
3. build this package
```bash
cd arm_ws && catkin build final_v1 && source devel/setup.bash
```
4. \+ 5. launch dwe driver + axis_grasp 
```bash
roslaunch dwe_stereo_camera_driver ikan.launch
```
```bash
roslaunch axis_grasp axis_grasp.launch
```
```bash
rosrun dwe_stereo_camera_driver replay_depth_image.py
```
```bash
_range_topic:=/ikan/explore3d/depth_image   _x0:=308 _y0:=308 _x1:=521 _y1:=493
```
6. launch this mission
```bash
roslaunch final_v1 pick.launch
```
7. send command to arm
```bash
rosservice call /pick/start
```
8. arm attempts to grab handle
```bash
rostopic echo /pick/state
rosservice call /pick/stop       
rosservice call /pick/reset
rosrun final_v1 arm.py home
```
