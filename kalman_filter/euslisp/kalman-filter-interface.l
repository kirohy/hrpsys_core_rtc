(require "package://jaxon_ros_interface/euslisp/rtm-ros-robot-interface.l")
(ros::load-ros-manifest "kalman_filter")
(instance rtm-ros-robot-interface :define-all-ROSBridge-srv-methods :ros-pkg-name "kalman_filter")

;; KalmanFilterService
(def-set-get-param-method
  'kalman_filter::kalman_filter_KalmanFilter2Service_KalmanFilterParam
  :set-kalman-filter-param :get-kalman-filter-param :get-kalman-filter-param-arguments
  :kalmanfilter2service_setkalmanfilterparam :kalmanfilter2service_getkalmanfilterparam)

(defmethod rtm-ros-robot-interface
  (:get-kalman-filter-algorithm
   ()
   "Get KalmanFilter Algorithm as Euslisp symbol."
   (send self :get-idl-enum-values
         (send (send self :get-kalman-filter-param) :kf_algorithm)
         "KALMAN_FILTER::KALMAN_FILTER_KALMANFILTER2SERVICE_KFALGORITHM"))
)

