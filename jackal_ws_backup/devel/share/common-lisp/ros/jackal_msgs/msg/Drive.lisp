; Auto-generated. Do not edit!


(cl:in-package jackal_msgs-msg)


;//! \htmlinclude Drive.msg.html

(cl:defclass <Drive> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (drivers
    :reader drivers
    :initarg :drivers
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Drive (<Drive>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Drive>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Drive)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jackal_msgs-msg:<Drive> is deprecated: use jackal_msgs-msg:Drive instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <Drive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:mode-val is deprecated.  Use jackal_msgs-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'drivers-val :lambda-list '(m))
(cl:defmethod drivers-val ((m <Drive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:drivers-val is deprecated.  Use jackal_msgs-msg:drivers instead.")
  (drivers m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Drive>)))
    "Constants for message type '<Drive>"
  '((:MODE_VELOCITY . 0)
    (:MODE_PWM . 1)
    (:MODE_EFFORT . 2)
    (:MODE_NONE . -1)
    (:LEFT . 0)
    (:RIGHT . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Drive)))
    "Constants for message type 'Drive"
  '((:MODE_VELOCITY . 0)
    (:MODE_PWM . 1)
    (:MODE_EFFORT . 2)
    (:MODE_NONE . -1)
    (:LEFT . 0)
    (:RIGHT . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Drive>) ostream)
  "Serializes a message object of type '<Drive>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'drivers))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Drive>) istream)
  "Deserializes a message object of type '<Drive>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:setf (cl:slot-value msg 'drivers) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'drivers)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Drive>)))
  "Returns string type for a message object of type '<Drive>"
  "jackal_msgs/Drive")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Drive)))
  "Returns string type for a message object of type 'Drive"
  "jackal_msgs/Drive")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Drive>)))
  "Returns md5sum for a message object of type '<Drive>"
  "601cf097cd47c174590c366c6ddd5fb3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Drive)))
  "Returns md5sum for a message object of type 'Drive"
  "601cf097cd47c174590c366c6ddd5fb3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Drive>)))
  "Returns full string definition for message of type '<Drive>"
  (cl:format cl:nil "# This message represents a low-level motor command to Jackal.~%~%# Command units dependent on the value of this field~%int8 MODE_VELOCITY=0   # velocity command (rad/s of wheels)~%int8 MODE_PWM=1        # proportion of full voltage command [-1.0..1.0]~%int8 MODE_EFFORT=2     # torque command (Nm)~%int8 MODE_NONE=-1      # no control, commanded values ignored.~%int8 mode~%~%# Units as above, +ve direction propels chassis forward.~%int8 LEFT=0~%int8 RIGHT=1~%float32[2] drivers~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Drive)))
  "Returns full string definition for message of type 'Drive"
  (cl:format cl:nil "# This message represents a low-level motor command to Jackal.~%~%# Command units dependent on the value of this field~%int8 MODE_VELOCITY=0   # velocity command (rad/s of wheels)~%int8 MODE_PWM=1        # proportion of full voltage command [-1.0..1.0]~%int8 MODE_EFFORT=2     # torque command (Nm)~%int8 MODE_NONE=-1      # no control, commanded values ignored.~%int8 mode~%~%# Units as above, +ve direction propels chassis forward.~%int8 LEFT=0~%int8 RIGHT=1~%float32[2] drivers~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Drive>))
  (cl:+ 0
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'drivers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Drive>))
  "Converts a ROS message object to a list"
  (cl:list 'Drive
    (cl:cons ':mode (mode msg))
    (cl:cons ':drivers (drivers msg))
))
