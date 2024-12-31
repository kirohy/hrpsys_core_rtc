(require "package://jaxon_ros_interface/euslisp/rtm-ros-robot-interface.l")
(ros::load-ros-manifest "sequence_player")
(instance rtm-ros-robot-interface :define-all-ROSBridge-srv-methods :ros-pkg-name "sequence_player")

(defmethod rtm-ros-robot-interface
  (:set-interpolation-mode
   (interpolation-mode)
   "Set interpolation mode for SequencePlayer."
   (if (integerp interpolation-mode)
       interpolation-mode
     (let ((im (read-from-string (format nil "SEQUENCE_PLAYER::SEQUENCE_PLAYER_SEQUENCEPLAYERSERVICE_INTERPOLATIONMODE::*~A*" (string-downcase interpolation-mode)))))
       (if (boundp im)
           (send self :sequenceplayer2service_setinterpolationmode :i_mode_ (eval im))
         (error ";; no such interpolation-mode ~A in :set-interpolation-mode~%" interpolation-mode)))
     ))
  (:set-base-pose
   (&optional base-coords (tm 0.1))
   (warn ";; :set-base-pose is deprecated. Use :set-base-coords~%")
   (unless base-coords
     (setq base-coords (send robot :worldcoords)))
   (send self :sequenceplayer2service_setbasepos :pos (scale 0.001 (send base-coords :pos)) :tm tm)
   (send self :sequenceplayer2service_setbaserpy :rpy (coerce (reverse (car (send base-coords :rpy-angle))) float-vector) :tm tm))
  (:set-base-coords
   (base-coords tm)
   "Set base coordinates in the world frame.
    base-coords is Euslisp coords and tm is [ms]."
   (send self :set-base-pos (send base-coords :worldpos) tm)
   (send self :set-base-rpy (coerce (reverse (car (rpy-angle (send base-coords :worldrot)))) float-vector) tm)
   )
  (:set-base-pos
   (base-pos tm)
   "Set base pos in the world frame.
    base-pos is [mm] and tm is [ms]."
   (send self :sequenceplayer2service_setbasepos :pos (scale 0.001 base-pos) :tm (* 0.001 tm)) ;; [mm]->[m], [ms]->[s]
   )
  (:set-base-rpy
   (base-rpy tm)
   "Set base rpy in the world frame.
    base-rpy is [rad] and tm is [ms]."
   (send self :sequenceplayer2service_setbaserpy :rpy base-rpy :tm (* 0.001 tm)) ;; [ms]->[s]
   )
  (:wait-interpolation-of-group
   (groupname)
   "Wait interpolation of group.
    !!This method is not recommended, please use :wait-interpolation method like (send *ri* :wait-interpolation :head-controller).!!"
   (send self :sequenceplayer2service_waitinterpolationofgroup :gname groupname))
  (:add-joint-group
   (groupname &optional (jnames
                         (if (find-method self (read-from-string (format nil ":~A-controller" (string-downcase groupname))))
                             (cdr (assoc :joint-names (car (send self (read-from-string (format nil ":~A-controller" (string-downcase groupname))))))))))
   "Add joint group for SequencePlayer.
    groupname is joint group name such as rarm or lleg.
    jnames is list of joint name."
   (unless jnames
     (error ";; jnames argument is require in :add-joint-group~%"))
   (send self :sequenceplayer2service_addjointgroup :gname groupname :jnames jnames))
  (:remove-joint-group
   (groupname)
   "Remove joint group for SequencePlayer.
    groupname is joint group name such as rarm or lleg."
   (send self :sequenceplayer2service_removejointgroup :gname groupname))
  (:set-joint-angles-of-group
   (groupname av tm)
   "Set joint angles of group.
    !!This method is not recommended, please use :angle-vector method like (send *ri* :angle-vector (send *robot* :angle-vector) 2000 :head-controller).!!"
   (send self :sequenceplayer2service_setjointanglesofgroup :gname groupname :jvs av :tm tm))
  (:load-pattern
   (basename &optional (tm 0.0))
   "Load pattern files, such as xx.pos and xx.waist.
    For pattern file definitions, please see loadPattern in SequencePlayer documentation in hrpsys-base API Doc."
   (send self :sequenceplayer2service_loadpattern :basename basename :tm tm))
  (:wait-interpolation-seq
   ()
   "Directly call SequencePlayer waitInterpolation.
    This can be used for force/moment interpolation."
   (send self :sequenceplayer2service_waitinterpolation))
  (:sync-controller
   (controller &optional (interpolation-time 1000) (blockp t))
   (let* ((controller-info (car (send self controller)))
          (groupname (cdr (assoc :group-name controller-info)))
          (jointnames (cdr (assoc :joint-names controller-info)))
          (current-reference (send self :state :reference-vector)))
     (unless current-reference
       (error ";; cannot get reference-vector in :sync-controller~%")
       )
     (warn "sync controller ~A~%" controller)
     (send self :angle-vector current-reference interpolation-time controller)
     (when blockp
       (send self :wait-interpolation)
       )
     (send self :angle-vector current-reference interpolation-time :default-controller)
     (when blockp
       (send self :wait-interpolation)
       )
     (send self :remove-joint-group groupname)
     (send self :wait-interpolation) ;; wait until removing
     (send self :add-joint-group groupname jointnames)
     )
   )
#| ;; angle group sample
  (send *ri* :add-joint-group "larm" (send-all (send *robot* :larm :joint-list) :name))
  (send *ri* :set-jointangles-of-group "larm" (scale (/ pi 180.0) (send *robot* :larm :angle-vector)) 4.0)
  (send *ri* :waitinterpolation-of-group "larm")
|#
  (:set-ref-forces-moments
   (force-list moment-list tm)
   "Set reference wrenches. wrench-list is list of wrench ([N],[Nm]) for all end-effectors. tm is interpolation time [ms]."
   (send self :sequenceplayer2service_setwrenches :wrenches (apply #'concatenate float-vector (mapcan #'(lambda (f m) (list f m)) force-list moment-list)) :tm (* 1e-3 tm)) ;; [ms]->[s]
   )
  (:set-ref-forces
   (force-list tm &key (update-robot-state t))
   "Set reference forces. force-list is list of force ([N]) for all end-effectors. tm is interpolation time [ms]."
   (if update-robot-state (send self :state))
   (send self :set-ref-forces-moments force-list (send self :reference-moment-vector) tm)
   )
  (:set-ref-moments
   (moment-list tm &key (update-robot-state t))
   "Set reference moments. moment-list is list of moment ([Nm]) for all end-effectors. tm is interpolation time [ms]."
   (if update-robot-state (send self :state))
   (send self :set-ref-forces-moments (send self :reference-force-vector) moment-list tm)
   )
  (:set-ref-force-moment
   (force moment tm &optional (limb :arms) &key (update-robot-state t))
   "Set reference force [N] and moment [Nm]. tm is interpolation time [ms].
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs."
   (if update-robot-state (send self :state))
   (let ((limbs (case limb
		  (:arms (list :rarm :larm))
		  (:legs (list :rleg :lleg))
		  (t (list limb)))))
     (send self :set-ref-forces-moments
           (mapcar #'(lambda (fs rfv)
                       (if (find-if #'(lambda (l) (equal fs (car (send robot l :force-sensors)))) limbs)
                           force rfv))
                   (send robot :force-sensors) (send self :reference-force-vector))
	   (mapcar #'(lambda (fs rfv)
                       (if (find-if #'(lambda (l) (equal fs (car (send robot l :force-sensors)))) limbs)
                           moment rfv))
                   (send robot :force-sensors) (send self :reference-moment-vector))
           tm)
     ))
  (:set-ref-force
   (force tm &optional (limb :arms) &key (update-robot-state t))
   "Set reference force [N]. tm is interpolation time [ms].
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs."
   (if update-robot-state (send self :state))
   (let ((limbs (case limb
                      (:arms (list :rarm :larm))
                      (:legs (list :rleg :lleg))
                      (t (list limb)))))
     (send self :set-ref-forces
           (mapcar #'(lambda (fs rfv)
                       (if (find-if #'(lambda (l) (equal fs (car (send robot l :force-sensors)))) limbs)
                           force rfv))
                   (send robot :force-sensors) (send self :reference-force-vector))
           tm)
     ))
  (:set-ref-moment
   (moment tm &optional (limb :arms) &key (update-robot-state t))
   "Set reference moment [Nm]. tm is interpolation time [ms].
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs."
   (if update-robot-state (send self :state))
   (let ((limbs (case limb
                      (:arms (list :rarm :larm))
                      (:legs (list :rleg :lleg))
                      (t (list limb)))))
     (send self :set-ref-moments
           (mapcar #'(lambda (fs rfv)
                       (if (find-if #'(lambda (l) (equal fs (car (send robot l :force-sensors)))) limbs)
                           moment rfv))
                   (send robot :force-sensors) (send self :reference-moment-vector))
           tm)
     ))
  (:angle-vector-sequence-full
   (jpos tm
         &key
         (sequence-length (length jpos))
         (joint-length (length (car jpos)))
         (fsensor-length (length (send robot :force-sensors))) ;; "fsensor length = ee length" is assumed
         (vel (make-list sequence-length :initial-element (instantiate float-vector joint-length)))
         (torque (make-list sequence-length :initial-element (instantiate float-vector joint-length)))
         (root-coords (make-list sequence-length :initial-element (make-coords)))
         (acc (make-list sequence-length :initial-element (instantiate float-vector 3)))
         (zmp (make-list sequence-length :initial-element (instantiate float-vector 3)))
         (wrench (make-list sequence-length :initial-element (instantiate float-vector (* 6 fsensor-length)))) ;; wrench = 6dim
         (optional (make-list sequence-length :initial-element (instantiate float-vector (* 2 fsensor-length)))) ;; 2 is for contactState and controlSwingSupportTime
         (pos (send-all root-coords :worldpos))
         (rpy (mapcar #'(lambda (x) (reverse (car (rpy-angle (send x :worldrot))))) root-coords))
         (root-local-zmp (mapcar #'(lambda (zz cc) (send cc :inverse-transform-vector zz)) zmp root-coords))
         )
   "Call service for setJointAnglesSequenceFull. Definition of each sequence is similar to sequence file of loadPattern.
    Arguments type:
     Required
      jpos: sequence of joint angles(float-vector) [deg],  (list av0 av1 ... avn)
      tm: sequence of duration(float) [ms],  (list tm0 tm1 ... tmn)
     Key
      vel: sequence of joint angular velocities(float-vector) [deg/s],  (list vel0 vel1 ... veln)
      torque: sequence of torques(float-vector) [Nm],  (list torque0 torque1 ... torquen)
      root-coords: sequence of waist(root-link) coords in the world frame. (list rc0 rc1 ... rcn). Origin coords by default.
      acc: sequence of waist acc(float-vector) [m/s^2],  (list acc0 acc1 ... accn)
      zmp: sequence of zmp in the world frame (float-vector) [mm],  (list zmp0 zmp1 ... zmpn). Zero by default.
      wrench: sequence of wrench(float-vector) [N, Nm] for all fsensors,  (list wrench0 wrench1 ... wrenchn)
      optional: sequence of optional(float-vector) [],  (list optional0 optional1 ... optionaln)
     Not required (calculated from other arguments by default), therefore users need not to use these arguments.
      pos: sequence of waist pos(float-vector) [mm] in the world frame,  (list pos0 pos1 ... posn). If root-coords is specified, calculated from root-coords and do not set pos.
      rpy: sequence of waist rpy(float-vector) [rad] in the world frame,  (list rpy0 rpy1 ... rpyn). If root-coords is specified, calculated from root-coords and do not set rpy.
      root-local-zmp: sequence of zmp in the waist(root-link) frame (float-vector) [mm],  (list zmp0 zmp1 ... zmpn). If root-coords and zmp are specified, calculated from root-coords and zmp and do not set root-local-zmp."
   (let* ((jvss (instance std_msgs::float64multiarray :init))
          (vels (instance std_msgs::float64multiarray :init))
          (torques (instance std_msgs::float64multiarray :init))
          (poss (instance std_msgs::float64multiarray :init))
          (rpys (instance std_msgs::float64multiarray :init))
          (accs (instance std_msgs::float64multiarray :init))
          (zmps (instance std_msgs::float64multiarray :init))
          (wrenchs (instance std_msgs::float64multiarray :init))
          (optionals (instance std_msgs::float64multiarray :init))
          (tms (coerce (mapcar #'(lambda (ttm) (* 1e-3 ttm)) tm) float-vector))) ;; [ms]->[s], list -> float-vector
     ;; jvss
     (let ((jvs (apply #'matrix (mapcar #'(lambda (x) (map float-vector #'deg2rad x)) jpos)))) ;; [deg] -> [rad]
       (send jvss :layout :dim (list
                                (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions jvs)) :stride (* (nth 0 (array-dimensions jvs)) (nth 1 (array-dimensions jvs))))
                                (instance std_msgs::multiArraydimension :init :label "jvs length" :size (nth 1 (array-dimensions jvs)) :stride (nth 1 (array-dimensions jvs)))))
       (send jvss :data (array-entity jvs)))
     ;; vels
     (setq vel (apply #'matrix (mapcar #'(lambda (x) (map float-vector #'deg2rad x)) vel))) ;; [deg/s] -> [rad/s]
     (send vels :layout :dim (list
                              (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions vel)) :stride (* (nth 0 (array-dimensions vel)) (nth 1 (array-dimensions vel))))
                              (instance std_msgs::multiArraydimension :init :label "vel length" :size (nth 1 (array-dimensions vel)) :stride (nth 1 (array-dimensions vel)))))
     (send vels :data (array-entity vel))
     ;; torques
     (setq torque (apply #'matrix torque))
     (send torques :layout :dim (list
                                 (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions torque)) :stride (* (nth 0 (array-dimensions torque)) (nth 1 (array-dimensions torque))))
                                 (instance std_msgs::multiArraydimension :init :label "torque length" :size (nth 1 (array-dimensions torque)) :stride (nth 1 (array-dimensions torque)))))
     (send torques :data (array-entity torque))
     ;; poss
     (setq pos (apply #'matrix (mapcar #'(lambda (x) (scale 1e-3 x)) pos))) ;; [mm] -> [m]
     (send poss :layout :dim (list
                              (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions pos)) :stride (* (nth 0 (array-dimensions pos)) (nth 1 (array-dimensions pos))))
                              (instance std_msgs::multiArraydimension :init :label "pos length" :size (nth 1 (array-dimensions pos)) :stride (nth 1 (array-dimensions pos)))))
     (send poss :data (array-entity pos))
     ;; rpys
     (setq rpy (apply #'matrix rpy))
     (send rpys :layout :dim (list
                              (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions rpy)) :stride (* (nth 0 (array-dimensions rpy)) (nth 1 (array-dimensions rpy))))
                              (instance std_msgs::multiArraydimension :init :label "rpy length" :size (nth 1 (array-dimensions rpy)) :stride (nth 1 (array-dimensions rpy)))))
     (send rpys :data (array-entity rpy))
     ;; accs
     (setq acc (apply #'matrix acc))
     (send accs :layout :dim (list
                              (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions acc)) :stride (* (nth 0 (array-dimensions acc)) (nth 1 (array-dimensions acc))))
                              (instance std_msgs::multiArraydimension :init :label "acc length" :size (nth 1 (array-dimensions acc)) :stride (nth 1 (array-dimensions acc)))))
     (send accs :data (array-entity acc))
     ;; zmps
     (setq zmp (apply #'matrix (mapcar #'(lambda (x) (scale 1e-3 x)) root-local-zmp))) ;; [mm] -> [m]
     (send zmps :layout :dim (list
                              (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions zmp)) :stride (* (nth 0 (array-dimensions zmp)) (nth 1 (array-dimensions zmp))))
                              (instance std_msgs::multiArraydimension :init :label "zmp length" :size (nth 1 (array-dimensions zmp)) :stride (nth 1 (array-dimensions zmp)))))
     (send zmps :data (array-entity zmp))
     ;; wrenchs
     (setq wrench (apply #'matrix wrench))
     (send wrenchs :layout :dim (list
                                 (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions wrench)) :stride (* (nth 0 (array-dimensions wrench)) (nth 1 (array-dimensions wrench))))
                                 (instance std_msgs::multiArraydimension :init :label "wrench length" :size (nth 1 (array-dimensions wrench)) :stride (nth 1 (array-dimensions wrench)))))
     (send wrenchs :data (array-entity wrench))
     ;; optionals
     (setq optional (apply #'matrix optional))
     (send optionals :layout :dim (list
                                   (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions optional)) :stride (* (nth 0 (array-dimensions optional)) (nth 1 (array-dimensions optional))))
                                   (instance std_msgs::multiArraydimension :init :label "optional length" :size (nth 1 (array-dimensions optional)) :stride (nth 1 (array-dimensions optional)))))
     (send optionals :data (array-entity optional))

     (send self :sequenceplayer2service_setjointanglessequencefull :jvss jvss :vels vels :torques torques :poss poss :rpys rpys :accs accs :zmps zmps :wrenchs wrenchs :optionals optionals :tms tms)
     )
   )
  )

