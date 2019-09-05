; Auto-generated. Do not edit!


(cl:in-package jackal_msgs-msg)


;//! \htmlinclude Feedback.msg.html

(cl:defclass <Feedback> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (drivers
    :reader drivers
    :initarg :drivers
    :type (cl:vector jackal_msgs-msg:DriveFeedback)
   :initform (cl:make-array 2 :element-type 'jackal_msgs-msg:DriveFeedback :initial-element (cl:make-instance 'jackal_msgs-msg:DriveFeedback)))
   (pcb_temperature
    :reader pcb_temperature
    :initarg :pcb_temperature
    :type cl:float
    :initform 0.0)
   (mcu_temperature
    :reader mcu_temperature
    :initarg :mcu_temperature
    :type cl:float
    :initform 0.0)
   (commanded_mode
    :reader commanded_mode
    :initarg :commanded_mode
    :type cl:fixnum
    :initform 0)
   (actual_mode
    :reader actual_mode
    :initarg :actual_mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Feedback (<Feedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Feedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Feedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jackal_msgs-msg:<Feedback> is deprecated: use jackal_msgs-msg:Feedback instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:header-val is deprecated.  Use jackal_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'drivers-val :lambda-list '(m))
(cl:defmethod drivers-val ((m <Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:drivers-val is deprecated.  Use jackal_msgs-msg:drivers instead.")
  (drivers m))

(cl:ensure-generic-function 'pcb_temperature-val :lambda-list '(m))
(cl:defmethod pcb_temperature-val ((m <Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:pcb_temperature-val is deprecated.  Use jackal_msgs-msg:pcb_temperature instead.")
  (pcb_temperature m))

(cl:ensure-generic-function 'mcu_temperature-val :lambda-list '(m))
(cl:defmethod mcu_temperature-val ((m <Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:mcu_temperature-val is deprecated.  Use jackal_msgs-msg:mcu_temperature instead.")
  (mcu_temperature m))

(cl:ensure-generic-function 'commanded_mode-val :lambda-list '(m))
(cl:defmethod commanded_mode-val ((m <Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:commanded_mode-val is deprecated.  Use jackal_msgs-msg:commanded_mode instead.")
  (commanded_mode m))

(cl:ensure-generic-function 'actual_mode-val :lambda-list '(m))
(cl:defmethod actual_mode-val ((m <Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:actual_mode-val is deprecated.  Use jackal_msgs-msg:actual_mode instead.")
  (actual_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Feedback>) ostream)
  "Serializes a message object of type '<Feedback>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'drivers))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pcb_temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mcu_temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'commanded_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'actual_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Feedback>) istream)
  "Deserializes a message object of type '<Feedback>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'drivers) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'drivers)))
    (cl:dotimes (i 2)
    (cl:setf (cl:aref vals i) (cl:make-instance 'jackal_msgs-msg:DriveFeedback))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pcb_temperature) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mcu_temperature) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'commanded_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'actual_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Feedback>)))
  "Returns string type for a message object of type '<Feedback>"
  "jackal_msgs/Feedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Feedback)))
  "Returns string type for a message object of type 'Feedback"
  "jackal_msgs/Feedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Feedback>)))
  "Returns md5sum for a message object of type '<Feedback>"
  "3bdabb0ef46338ee5672d1b82220ab49")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Feedback)))
  "Returns md5sum for a message object of type 'Feedback"
  "3bdabb0ef46338ee5672d1b82220ab49")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Feedback>)))
  "Returns full string definition for message of type '<Feedback>"
  (cl:format cl:nil "# This message represents high-frequency feedback from the MCU,~%# as necessary to support closed-loop control and thermal monitoring.~%# Default publish frequency is 50Hz.~%~%Header header~%~%DriveFeedback[2] drivers~%~%# Temperatures ~%float32 pcb_temperature~%float32 mcu_temperature~%~%# Commanded control mode, use the TYPE_ constants from jackal_msgs/Drive.~%int8 commanded_mode~%~%# Actual control mode. This may differ from the commanded in cases where~%# the motor enable is off, the motors are in over-current, etc.~%int8 actual_mode~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: jackal_msgs/DriveFeedback~%# This message represents feedback data from a single drive unit (driver + motor).~%~%# Current flowing from battery into the MOSFET bridge.~%float32 current~%~%# Instantaneous duty cycle of MOSFET bridge.~%float32 duty_cycle~%~%# Temperatures measured in the MOSFET bridge and on the motor casing, in deg C.~%float32 bridge_temperature~%float32 motor_temperature~%~%# Encoder data~%float32 measured_velocity   # rad/s~%float32 measured_travel     # rad~%~%# True if the underlying driver chip reports a fault condition.~%bool driver_fault~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Feedback)))
  "Returns full string definition for message of type 'Feedback"
  (cl:format cl:nil "# This message represents high-frequency feedback from the MCU,~%# as necessary to support closed-loop control and thermal monitoring.~%# Default publish frequency is 50Hz.~%~%Header header~%~%DriveFeedback[2] drivers~%~%# Temperatures ~%float32 pcb_temperature~%float32 mcu_temperature~%~%# Commanded control mode, use the TYPE_ constants from jackal_msgs/Drive.~%int8 commanded_mode~%~%# Actual control mode. This may differ from the commanded in cases where~%# the motor enable is off, the motors are in over-current, etc.~%int8 actual_mode~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: jackal_msgs/DriveFeedback~%# This message represents feedback data from a single drive unit (driver + motor).~%~%# Current flowing from battery into the MOSFET bridge.~%float32 current~%~%# Instantaneous duty cycle of MOSFET bridge.~%float32 duty_cycle~%~%# Temperatures measured in the MOSFET bridge and on the motor casing, in deg C.~%float32 bridge_temperature~%float32 motor_temperature~%~%# Encoder data~%float32 measured_velocity   # rad/s~%float32 measured_travel     # rad~%~%# True if the underlying driver chip reports a fault condition.~%bool driver_fault~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Feedback>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'drivers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Feedback>))
  "Converts a ROS message object to a list"
  (cl:list 'Feedback
    (cl:cons ':header (header msg))
    (cl:cons ':drivers (drivers msg))
    (cl:cons ':pcb_temperature (pcb_temperature msg))
    (cl:cons ':mcu_temperature (mcu_temperature msg))
    (cl:cons ':commanded_mode (commanded_mode msg))
    (cl:cons ':actual_mode (actual_mode msg))
))
