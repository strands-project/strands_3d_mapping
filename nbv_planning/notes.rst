Projection matrix from Morse:
```
chris@heatwave:~/ros_workspaces/planning_ws/src/scitos_3d_mapping/nbv_planning/scripts$ rostopic echo -n 1 /head_xtion/depth_registered/camera_info
header:
  seq: 0
  stamp:
    secs: 1452267504
    nsecs: 684045302
  frame_id: head_xtion_rgb_optical_frame
height: 480
width: 640
distortion_model: plumb_bob
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi:
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
---
chris@heatwave:~/ros_workspaces/planning_ws/src/scitos_3d_mapping/nbv_planning/scripts$
```