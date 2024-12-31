(require "package://jaxon_ros_interface/euslisp/rtm-ros-robot-interface.l")
(ros::load-ros-manifest "remove_force_sensor_link_offset")
(instance rtm-ros-robot-interface :define-all-ROSBridge-srv-methods :ros-pkg-name "remove_force_sensor_link_offset")

(def-set-get-param-method 'remove_force_sensor_link_offset::remove_force_sensor_link_offset_RemoveForceSensorLinkOffset2Service_forcemomentOffsetParam
  :raw-set-forcemoment-offset-param :raw-get-forcemoment-offset-param :get-forcemoment-offset-param-arguments
  :removeforcesensorlinkoffset2service_setforcemomentoffsetparam :removeforcesensorlinkoffset2service_getforcemomentoffsetparam
  :optional-args (list :name 'name))

(defmethod rtm-ros-robot-interface
  (:zero-set-forcemoment-offset-param
   (limb)
   "Set RemoveForceSensorLinkOffset's params offset to zero."
   (send self :set-forcemoment-offset-param limb :force-offset #f(0 0 0) :moment-offset #f(0 0 0) :link-offset-centroid #f(0 0 0) :link-offset-mass 0)
   )
  (:set-forcemoment-offset-param
   (limb &rest args)
   "Set RemoveForceSensorLinkOffset params for given limb.
    For arguments, please see (send *ri* :get-forcemoment-offset-param-arguments)."
   (send* self :force-sensor-method
         limb
         #'(lambda (name &rest _args)
             (send* self :raw-set-forcemoment-offset-param (send (car (send robot name :force-sensors)) :name) _args))
         :set-forcemoment-offset-param
         args))
  (:get-forcemoment-offset-param
   (limb)
   "Get RemoveForceSensorLinkOffset params for given limb."
   (send self :force-sensor-method
         limb
         #'(lambda (name &rest _args)
             (send self :raw-get-forcemoment-offset-param (send (car (send robot name :force-sensors)) :name)))
         :get-forcemoment-offset-param))
  (:load-forcemoment-offset-param
   (fname &key (set-offset t))
   "Load RemoveForceSensorLinkOffset params from fname (file path)."
   (mapcar #'(lambda (x)
               (send* self :set-forcemoment-offset-param (car x)
                      (if set-offset
                          (cdr x)
                        (list :link-offset-mass (cadr (memq :link-offset-mass (cdr x)))
                              :link-offset-centroid (cadr (memq :link-offset-centroid (cdr x)))))))
           (with-open-file
            (f fname :direction :input)
            (read f nil nil)))
   )
  (:load-forcemoment-offset-params
   (filename)
   "Load RMFO offset parameters from parameter file.
    This method corresponds to RemoveForceSensorLinkOffset loadForceMomentOffsetParams()."
   (send self :removeforcesensorlinkoffset2service_loadforcemomentoffsetparams :filename filename)
   )
  (:dump-forcemoment-offset-params
   (filename &key (set-robot-date-string t))
   "Save all RMFO offset parameters.
    This method corresponds to RemoveForceSensorLinkOffset dumpForceMomentOffsetParams().
    If set-robot-date-string is t, filename includes date string and robot name. By default, set-robot-date-string is t."
   (send self :removeforcesensorlinkoffset2service_dumpforcemomentoffsetparams :filename (format nil "~A~A" filename (if set-robot-date-string (format nil "_~A" (send self :get-robot-date-string)) "")))
   )
  (:remove-force-sensor-offset-rmfo
   (&key (limbs) ((:time tm) 8.0))
   "remove offsets on sensor outputs form force/torque sensors.
    Sensor offsets (force_offset and moment_offset in ForceMomentOffsetParam) are calibrated.
    Please keep the robot static and make sure that robot's sensors do not contact with any objects.
    Argument:
      limbs is list of sensor names to be calibrated.
      If not specified, all sensors are calibrated by default.
      time is duration of calibration[s]. 8.0[s] by default.
    Return:
      t if set successfully, nil otherwise"
   (send self :removeforcesensorlinkoffset2service_removeforcesensoroffset
         :names
         (mapcar #'(lambda (limb) (send (car (send robot limb :force-sensors)) :name)) limbs)
         :tm tm))
  (:remove-force-sensor-offset-rmfo-arms
   (&key ((:time tm) 8.0))
   "Remove force and moment offset for :rarm and :larm.
    time is duration of calibration[s]. 8.0[s] by default."
   (send self :remove-force-sensor-offset-rmfo :limbs '(:rarm :larm) :time tm))
  (:remove-force-sensor-offset-rmfo-legs
   (&key ((:time tm) 8.0))
   "Remove force and moment offset for :rleg and :lleg.
    time is duration of calibration[s]. 8.0[s] by default."
   (send self :remove-force-sensor-offset-rmfo :limbs '(:rleg :lleg) :time tm))
  ;; Deprecated
  (:reset-force-moment-offset-arms
   (&key ((:time tm) 0.1))
   "time[s]"
   (warning-message 1 ";; !!!!!!!!~%")
   (warning-message 1 ";; !! Warning, :reset-force-moment-offset-arms is deprecated. Please use (send *ri* :remove-force-sensor-offset-rmfo-arms :time 0.1)~%")
   (warning-message 1 ";; !!!!!!!!~%")
   (send self :remove-force-sensor-offset-rmfo-arms :time tm))
  (:reset-force-moment-offset-legs
   (&key ((:time tm) 0.1))
   "time[s]"
   (warning-message 1 ";; !!!!!!!!~%")
   (warning-message 1 ";; !! Warning, :reset-force-moment-offset-legs is deprecated. Please use (send *ri* :remove-force-sensor-offset-rmfo-legs :time 0.1)~%")
   (warning-message 1 ";; !!!!!!!!~%")
   (send self :remove-force-sensor-offset-rmfo-legs :time tm))
  (:reset-force-moment-offset
   (limbs)
   "Remove force and moment offsets. limbs should be list of limb symbol name."
   (send self :_reset-force-moment-offset limbs :force)
   (send self :_reset-force-moment-offset limbs :moment)
   )
  (:_reset-force-moment-offset
   (limbs f/m &key (itr 10))
   (let* ((params (mapcar #'(lambda (alimb) (send self :get-forcemoment-offset-param alimb)) limbs)))
     (labels ((calc-off
               (alimb)
               (send self (if (eq f/m :force) :off-force-vector :off-moment-vector) alimb))
              (get-avg-fm
               ()
               (let ((fm (mapcar #'(lambda (i)
                                     (send self :state)
                                     (mapcar #'(lambda (alimb) (send self (if (eq f/m :force) :off-force-vector :off-moment-vector) alimb)) limbs))
                                 (make-list itr))))
                 (mapcar #'(lambda (alimb)
                             (let ((idx (position alimb limbs)))
                               (vector-mean (mapcar #'(lambda (d) (elt d idx)) fm))))
                         limbs))))
       ;; estimate offsets
       (let* ((tmp-fm-offsets (mapcar #'(lambda (i)
                                          (send self :state)
                                          (mapcar #'calc-off limbs))
                                      (make-list itr)))
              (new-fm-offsets (mapcar #'(lambda (alimb)
                                          (let ((idx (position alimb limbs)))
                                            (vector-mean (mapcar #'(lambda (d) (elt d idx)) tmp-fm-offsets))))
                                      limbs))
              (org-fm-list (get-avg-fm)))
         ;; set offsets
         (mapcar #'(lambda (alimb new-fm-offset param)
                     (send self :set-forcemoment-offset-param alimb
                           (if (eq f/m :force) :force-offset :moment-offset)
                           (v+ (if (eq f/m :force)
                                   (send param :force_offset)
                                 (send param :moment_offset))
                               new-fm-offset)))
                 limbs new-fm-offsets params)
         (unix:usleep 10000)
         ;; check ;; compare sensor value before & after resetting
         (mapcar #'(lambda (alimb org-fm new-fm)
                     (format t ";; ~A error of ~A ;; ~A[~A] -> ~A[~A]~%"
                             (string-downcase f/m) alimb
                             (norm org-fm) (if (eq f/m :force) "N" "Nm")
                             (norm new-fm) (if (eq f/m :force) "N" "Nm")))
                 limbs org-fm-list (get-avg-fm))
         ))))
  )

