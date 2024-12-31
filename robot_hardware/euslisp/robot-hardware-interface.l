(require "package://jaxon_ros_interface/euslisp/rtm-ros-robot-interface.l")
(ros::load-ros-manifest "robot_hardware")
(instance rtm-ros-robot-interface :define-all-ROSBridge-srv-methods :ros-pkg-name "robot_hardware")

(defmethod rtm-ros-robot-interface
  (:set-servo-gain-percentage
   (name percentage)
   "Set servo gain percentage [0-100] with given name."
   (send self :robothardware2service_setservogainpercentage :name name :percentage percentage))
  (:remove-force-sensor-offset
   ()
   "Remove force sensor offset.
    This function takes 10[s]. Please keep the robot static and make sure that robot's sensors do not contact with any objects."
   (warning-message 1 ";; !!!!!!!!~%")
   (warning-message 1 ";; !! Warning, :remove-force-sensor-offset by RobotHardware is deprecated. Please use :remove-force-sensor-offset-rmfo~%")
   (warning-message 1 ";; !!!!!!!!~%")
   (send self :robothardware2service_removeforcesensoroffset))
  (:set-servo-error-limit
    (name limit)
    "Set RobotHardware servo error limit [rad] with given name."
    (send self :robothardware2service_setservoerrorlimit :name name :limit limit)
    )
  (:calibrate-inertia-sensor
   ()
   "Calibrate inetria sensor.
    This function takes 10[s]. Please keep the robot static."
   (send self :robothardware2service_calibrateInertiaSensor)
   )
  )

