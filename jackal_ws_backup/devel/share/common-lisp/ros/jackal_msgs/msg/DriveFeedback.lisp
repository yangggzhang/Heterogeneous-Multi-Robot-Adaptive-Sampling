; Auto-generated. Do not edit!


(cl:in-package jackal_msgs-msg)


;//! \htmlinclude DriveFeedback.msg.html

(cl:defclass <DriveFeedback> (roslisp-msg-protocol:ros-message)
  ((current
    :reader current
    :initarg :current
    :type cl:float
    :initform 0.0)
   (duty_cycle
    :reader duty_cycle
    :initarg :duty_cycle
    :type cl:float
    :initform 0.0)
   (bridge_temperature
    :reader bridge_temperature
    :initarg :bridge_temperature
    :type cl:float
    :initform 0.0)
   (motor_temperature
    :reader motor_temperature
    :initarg :motor_temperature
    :type cl:float
    :initform 0.0)
   (measured_velocity
    :reader measured_velocity
    :initarg :measured_velocity
    :type cl:float
    :initform 0.0)
   (measured_travel
    :reader measured_travel
    :initarg :measured_travel
    :type cl:float
    :initform 0.0)
   (driver_fault
    :reader driver_fault
    :initarg :driver_fault
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DriveFeedback (<DriveFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DriveFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DriveFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jackal_msgs-msg:<DriveFeedback> is deprecated: use jackal_msgs-msg:DriveFeedback instead.")))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <DriveFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:current-val is deprecated.  Use jackal_msgs-msg:current instead.")
  (current m))

(cl:ensure-generic-function 'duty_cycle-val :lambda-list '(m))
(cl:defmethod duty_cycle-val ((m <DriveFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:duty_cycle-val is deprecated.  Use jackal_msgs-msg:duty_cycle instead.")
  (duty_cycle m))

(cl:ensure-generic-function 'bridge_temperature-val :lambda-list '(m))
(cl:defmethod bridge_temperature-val ((m <DriveFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:bridge_temperature-val is deprecated.  Use jackal_msgs-msg:bridge_temperature instead.")
  (bridge_temperature m))

(cl:ensure-generic-function 'motor_temperature-val :lambda-list '(m))
(cl:defmethod motor_temperature-val ((m <DriveFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:motor_temperature-val is deprecated.  Use jackal_msgs-msg:motor_temperature instead.")
  (motor_temperature m))

(cl:ensure-generic-function 'measured_velocity-val :lambda-list '(m))
(cl:defmethod measured_velocity-val ((m <DriveFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:measured_velocity-val is deprecated.  Use jackal_msgs-msg:measured_velocity instead.")
  (measured_velocity m))

(cl:ensure-generic-function 'measured_travel-val :lambda-list '(m))
(cl:defmethod measured_travel-val ((m <DriveFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:measured_travel-val is deprecated.  Use jackal_msgs-msg:measured_travel instead.")
  (measured_travel m))

(cl:ensure-generic-function 'driver_fault-val :lambda-list '(m))
(cl:defmethod driver_fault-val ((m <DriveFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:driver_fault-val is deprecated.  Use jackal_msgs-msg:driver_fault instead.")
  (driver_fault m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DriveFeedback>) ostream)
  "Serializes a message object of type '<DriveFeedback>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'duty_cycle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bridge_temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motor_temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'measured_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'measured_travel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'driver_fault) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DriveFeedback>) istream)
  "Deserializes a message object of type '<DriveFeedback>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duty_cycle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bridge_temperature) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor_temperature) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'measured_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'measured_travel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'driver_fault) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DriveFeedback>)))
  "Returns string type for a message object of type '<DriveFeedback>"
  "jackal_msgs/DriveFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DriveFeedback)))
  "Returns string type for a message object of type 'DriveFeedback"
  "jackal_msgs/DriveFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DriveFeedback>)))
  "Returns md5sum for a message object of type '<DriveFeedback>"
  "8dd0b7a3cfa20cfc5c054ddd9763609b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DriveFeedback)))
  "Returns md5sum for a message object of type 'DriveFeedback"
  "8dd0b7a3cfa20cfc5c054ddd9763609b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DriveFeedback>)))
  "Returns full string definition for message of type '<DriveFeedback>"
  (cl:format cl:nil "# This message represents feedback data from a single drive unit (driver + motor).~%~%# Current flowing from battery into the MOSFET bridge.~%float32 current~%~%# Instantaneous duty cycle of MOSFET bridge.~%float32 duty_cycle~%~%# Temperatures measured in the MOSFET bridge and on the motor casing, in deg C.~%float32 bridge_temperature~%float32 motor_temperature~%~%# Encoder data~%float32 measured_velocity   # rad/s~%float32 measured_travel     # rad~%~%# True if the underlying driver chip reports a fault condition.~%bool driver_fault~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DriveFeedback)))
  "Returns full string definition for message of type 'DriveFeedback"
  (cl:format cl:nil "# This message represents feedback data from a single drive unit (driver + motor).~%~%# Current flowing from battery into the MOSFET bridge.~%float32 current~%~%# Instantaneous duty cycle of MOSFET bridge.~%float32 duty_cycle~%~%# Temperatures measured in the MOSFET bridge and on the motor casing, in deg C.~%float32 bridge_temperature~%float32 motor_temperature~%~%# Encoder data~%float32 measured_velocity   # rad/s~%float32 measured_travel     # rad~%~%# True if the underlying driver chip reports a fault condition.~%bool driver_fault~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DriveFeedback>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DriveFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'DriveFeedback
    (cl:cons ':current (current msg))
    (cl:cons ':duty_cycle (duty_cycle msg))
    (cl:cons ':bridge_temperature (bridge_temperature msg))
    (cl:cons ':motor_temperature (motor_temperature msg))
    (cl:cons ':measured_velocity (measured_velocity msg))
    (cl:cons ':measured_travel (measured_travel msg))
    (cl:cons ':driver_fault (driver_fault msg))
))
