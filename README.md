# Piper_Moveit_Servo

## パッケージ追加
-   piper_ros
-   moveit_servo（moveit2をcloneしてmoveit_servoパッケージだけ抜き出す．）
-   moveit_task_constructor（git clone）
-   moveit_visual_tools（git clone）

## 実行手順
```
ros2 launch piper_moveit_servo piper_servo_cpp_interface_demo.launch.py
ros2 run piper_moveit_servo piper_servo_keyboard_input
```
## 参考
- [Moveit_humble](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [西尾君のissue](https://github.com/open-rdc/harvesting_robot/issues/187)