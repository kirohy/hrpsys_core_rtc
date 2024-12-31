(require "package://jaxon_ros_interface/euslisp/rtm-ros-robot-interface.l")
(ros::load-ros-manifest "data_logger")
(instance rtm-ros-robot-interface :define-all-ROSBridge-srv-methods :ros-pkg-name "data_logger")

(defmethod rtm-ros-robot-interface
  (:save-log
   (fname &key (set-robot-date-string t) (make-directory nil))
   "Save log files as [fname].[component_name]_[dataport_name].
    This method corresponds to DataLogger save().
    If set-robot-date-string is t, filename includes date string and robot name. By default, set-robot-date-string is t.
    If make-directory is t and fname is /foo/bar/basename, make directory of /foo/bar/basename and log files will be saved with /foo/bar/basename/basename"
   (let ((basename
          (format nil "~A~A" fname (if set-robot-date-string (format nil "_~A" (send self :get-robot-date-string)) ""))))
     (when make-directory
       (let* ((pname (pathname basename))
              dirname
              (fname
               (if (send pname :type)
                   (concatenate string (send pname :name) "." (send pname :type))
                   (send pname :name))))
         (send pname :add-directory (list fname))
         (setq dirname (send pname :directory-string))
         (when (and dirname
                    (not (lisp::probe-file dirname)))
           (unix::mkdir dirname))
         (setq basename (send pname :namestring))
         ))
     (warn ";; save log to ~A~%" basename)
     (send self :datalogger2service_save :basename basename)
     ))
  ;; start log by clearing log
  (:start-log
   ()
   "Start logging.
    This method corresponds to DataLogger clear()."
   (send self :datalogger2service_clear))
  (:set-log-maxlength
   (&optional (maxlength 4000))
   "Set max log length.
    This method corresponds to DataLogger maxLength()."
   (send self :datalogger2service_maxlength :len maxlength))
  )

