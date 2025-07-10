# bento_teleop
The interface to control bento robots.
*Essentially a Joystick to whatever we need converter*

It is split up into two interfaces:
`drive_interface` does:
- interoperates with [bento_drive](https://github.com/Bento-Robotics/bento_drive)
- publish a Twist message to /cmd_vel, deduced from a max-velocity topic and joystick input
- which is reduced to zeroes while the button to control the arm is held
- call a std/Bool service that enables/disables motors at the press of a button
- an overcomplicated mess to control motor RPM directly via buttons (for schäufele's flippers)
- speed is influenced by throttle lever

`arm_interface` does:
- interoperates with [micro_rosso_2dof_arm](https://github.com/Bento-Robotics/micro_rosso_2dof_arm)
- publish a Point, which gives the arm relative position commands
- only controls the arm while you hold a button
- move the arm into it's home position when you press a button
- speed is influenced by throttle lever

see [Bento-Operation](https://github.com/Bento-Robotics/Bento-Operation) for a one-line docker command to begin this and other necessary teleoperation nodes
and [Bento-PC](https://github.com/Bento-Robotics/Bento-PC) for bash/terminal init scripts to make starting Bento-Operation a one-worder and automatically set up ROS