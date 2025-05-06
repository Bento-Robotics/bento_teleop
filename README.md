# bento_teleop
Essentially a Joystick to whatever we needed converter.

Regular teleop does:
- interoperates with [bento_drive](https://github.com/Bento-Robotics/bento_drive)
- publish a Twist message to /cmd_vel, deduced from a max-velocity topic and joystick input
- call a std/Bool service that enables motor control at the press of a button
- an overcomplicated mess to control motor RPM directly via buttons (for schäufele's flippers)
- speed is influenced by throttle lever

Arm teleop does:
- interoperates with [micro_rosso_2dof_arm](https://github.com/Bento-Robotics/micro_rosso_2dof_arm)
- publish a Point that gives the arm relative position commands
- only controls arm while you press a button
- move the arm into it's home position when you press a button
- speed is influenced by throttle lever

Crappy semi-spaghetti, but hey it works.
