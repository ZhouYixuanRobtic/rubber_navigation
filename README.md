## Rubber-Navigation

​	This is a navigation application including a base controller

​	 **Warning , the command in BaseController class is only suitable for my own STM32 program**

[TOC]

### Dependences

​	This project relies on my another project `Visual-Servo` and `move_base`; `yocs-velocity-smoother`;`robot_pose_ekg`;`Eigen3`

### INSTALL

​	Under your workspace source folder, clone two repository, then use `catkin_make`

```
git clone https://github.com/ZhouYixuanRobtic/rubber_navigation.git
git clone https://github.com/ZhouYixuanRobtic/Visual-Servo.git
```

### Usage

 - BaseOnly

   ```
   roslaunch rubber_navigation baseOnly.launch
   ```

- SimpleNav(no visual_servo action)

  ```
  roslaunch rubber_navigation simpleNav.launch
  ```

- GoalSaver