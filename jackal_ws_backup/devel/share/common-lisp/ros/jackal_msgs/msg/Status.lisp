; Auto-generated. Do not edit!


(cl:in-package jackal_msgs-msg)


;//! \htmlinclude Status.msg.html

(cl:defclass <Status> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (hardware_id
    :reader hardware_id
    :initarg :hardware_id
    :type cl:string
    :initform "")
   (mcu_uptime
    :reader mcu_uptime
    :initarg :mcu_uptime
    :type cl:real
    :initform 0)
   (connection_uptime
    :reader connection_uptime
    :initarg :connection_uptime
    :type cl:real
    :initform 0)
   (drivers_active
    :reader drivers_active
    :initarg :drivers_active
    :type cl:boolean
    :initform cl:nil)
   (driver_external_stop_present
    :reader driver_external_stop_present
    :initarg :driver_external_stop_present
    :type cl:boolean
    :initform cl:nil)
   (driver_external_stop_stopped
    :reader driver_external_stop_stopped
    :initarg :driver_external_stop_stopped
    :type cl:boolean
    :initform cl:nil)
   (measured_battery
    :reader measured_battery
    :initarg :measured_battery
    :type cl:float
    :initform 0.0)
   (measured_12v
    :reader measured_12v
    :initarg :measured_12v
    :type cl:float
    :initform 0.0)
   (measured_5v
    :reader measured_5v
    :initarg :measured_5v
    :type cl:float
    :initform 0.0)
   (drive_current
    :reader drive_current
    :initarg :drive_current
    :type cl:float
    :initform 0.0)
   (user_current
    :reader user_current
    :initarg :user_current
    :type cl:float
    :initform 0.0)
   (computer_current
    :reader computer_current
    :initarg :computer_current
    :type cl:float
    :initform 0.0)
   (total_current
    :reader total_current
    :initarg :total_current
    :type cl:float
    :initform 0.0)
   (total_current_peak
    :reader total_current_peak
    :initarg :total_current_peak
    :type cl:float
    :initform 0.0)
   (total_power_consumed
    :reader total_power_consumed
    :initarg :total_power_consumed
    :type cl:float
    :initform 0.0))
)

(cl:defclass Status (<Status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jackal_msgs-msg:<Status> is deprecated: use jackal_msgs-msg:Status instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:header-val is deprecated.  Use jackal_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'hardware_id-val :lambda-list '(m))
(cl:defmethod hardware_id-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:hardware_id-val is deprecated.  Use jackal_msgs-msg:hardware_id instead.")
  (hardware_id m))

(cl:ensure-generic-function 'mcu_uptime-val :lambda-list '(m))
(cl:defmethod mcu_uptime-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:mcu_uptime-val is deprecated.  Use jackal_msgs-msg:mcu_uptime instead.")
  (mcu_uptime m))

(cl:ensure-generic-function 'connection_uptime-val :lambda-list '(m))
(cl:defmethod connection_uptime-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:connection_uptime-val is deprecated.  Use jackal_msgs-msg:connection_uptime instead.")
  (connection_uptime m))

(cl:ensure-generic-function 'drivers_active-val :lambda-list '(m))
(cl:defmethod drivers_active-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:drivers_active-val is deprecated.  Use jackal_msgs-msg:drivers_active instead.")
  (drivers_active m))

(cl:ensure-generic-function 'driver_external_stop_present-val :lambda-list '(m))
(cl:defmethod driver_external_stop_present-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:driver_external_stop_present-val is deprecated.  Use jackal_msgs-msg:driver_external_stop_present instead.")
  (driver_external_stop_present m))

(cl:ensure-generic-function 'driver_external_stop_stopped-val :lambda-list '(m))
(cl:defmethod driver_external_stop_stopped-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:driver_external_stop_stopped-val is deprecated.  Use jackal_msgs-msg:driver_external_stop_stopped instead.")
  (driver_external_stop_stopped m))

(cl:ensure-generic-function 'measured_battery-val :lambda-list '(m))
(cl:defmethod measured_battery-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:measured_battery-val is deprecated.  Use jackal_msgs-msg:measured_battery instead.")
  (measured_battery m))

(cl:ensure-generic-function 'measured_12v-val :lambda-list '(m))
(cl:defmethod measured_12v-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:measured_12v-val is deprecated.  Use jackal_msgs-msg:measured_12v instead.")
  (measured_12v m))

(cl:ensure-generic-function 'measured_5v-val :lambda-list '(m))
(cl:defmethod measured_5v-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:measured_5v-val is deprecated.  Use jackal_msgs-msg:measured_5v instead.")
  (measured_5v m))

(cl:ensure-generic-function 'drive_current-val :lambda-list '(m))
(cl:defmethod drive_current-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:drive_current-val is deprecated.  Use jackal_msgs-msg:drive_current instead.")
  (drive_current m))

(cl:ensure-generic-function 'user_current-val :lambda-list '(m))
(cl:defmethod user_current-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:user_current-val is deprecated.  Use jackal_msgs-msg:user_current instead.")
  (user_current m))

(cl:ensure-generic-function 'computer_current-val :lambda-list '(m))
(cl:defmethod computer_current-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:computer_current-val is deprecated.  Use jackal_msgs-msg:computer_current instead.")
  (computer_current m))

(cl:ensure-generic-function 'total_current-val :lambda-list '(m))
(cl:defmethod total_current-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:total_current-val is deprecated.  Use jackal_msgs-msg:total_current instead.")
  (total_current m))

(cl:ensure-generic-function 'total_current_peak-val :lambda-list '(m))
(cl:defmethod total_current_peak-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:total_current_peak-val is deprecated.  Use jackal_msgs-msg:total_current_peak instead.")
  (total_current_peak m))

(cl:ensure-generic-function 'total_power_consumed-val :lambda-list '(m))
(cl:defmethod total_power_consumed-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jackal_msgs-msg:total_power_consumed-val is deprecated.  Use jackal_msgs-msg:total_power_consumed instead.")
  (total_power_consumed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Status>) ostream)
  "Serializes a message object of type '<Status>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'hardware_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'hardware_id))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'mcu_uptime)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'mcu_uptime) (cl:floor (cl:slot-value msg 'mcu_uptime)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'connection_uptime)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'connection_uptime) (cl:floor (cl:slot-value msg 'connection_uptime)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'drivers_active) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'driver_external_stop_present) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'driver_external_stop_stopped) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'measured_battery))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'measured_12v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'measured_5v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'drive_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'user_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'computer_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'total_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'total_current_peak))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'total_power_consumed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Status>) istream)
  "Deserializes a message object of type '<Status>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hardware_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'hardware_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mcu_uptime) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'connection_uptime) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:setf (cl:slot-value msg 'drivers_active) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'driver_external_stop_present) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'driver_external_stop_stopped) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'measured_battery) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'measured_12v) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'measured_5v) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'drive_current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'user_current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'computer_current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'total_current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'total_current_peak) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'total_power_consumed) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Status>)))
  "Returns string type for a message object of type '<Status>"
  "jackal_msgs/Status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Status)))
  "Returns string type for a message object of type 'Status"
  "jackal_msgs/Status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Status>)))
  "Returns md5sum for a message object of type '<Status>"
  "c851ebcf9a6e20b196bc7894e285b4f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Status)))
  "Returns md5sum for a message object of type 'Status"
  "c851ebcf9a6e20b196bc7894e285b4f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Status>)))
  "Returns full string definition for message of type '<Status>"
  (cl:format cl:nil "# This message represents Jackal's lower-frequency status updates~%# Default publish frequency is 1Hz.~%~%Header header~%~%# Commit of firmware source.~%string hardware_id~%~%# Times since MCU power-on and MCU rosserial connection, respectively.~%duration mcu_uptime~%duration connection_uptime~%~%# Monitoring the run/stop loop. Changes in these values trigger an immediate~%# publish, outside the ordinarily-scheduled 1Hz updates.~%bool drivers_active~%bool driver_external_stop_present~%bool driver_external_stop_stopped~%~%# Voltage rails, in volts~%# Averaged over the message period~%float32 measured_battery~%float32 measured_12v~%float32 measured_5v~%~%# Current senses available on platform, in amps.~%# Averaged over the message period~%float32 drive_current~%float32 user_current~%float32 computer_current~%float32 total_current~%~%# Highest total system current peak as measured in a 1ms window.~%float32 total_current_peak~%~%# Integration of all power consumption since MCU power-on, in watt-hours.~%float64 total_power_consumed ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Status)))
  "Returns full string definition for message of type 'Status"
  (cl:format cl:nil "# This message represents Jackal's lower-frequency status updates~%# Default publish frequency is 1Hz.~%~%Header header~%~%# Commit of firmware source.~%string hardware_id~%~%# Times since MCU power-on and MCU rosserial connection, respectively.~%duration mcu_uptime~%duration connection_uptime~%~%# Monitoring the run/stop loop. Changes in these values trigger an immediate~%# publish, outside the ordinarily-scheduled 1Hz updates.~%bool drivers_active~%bool driver_external_stop_present~%bool driver_external_stop_stopped~%~%# Voltage rails, in volts~%# Averaged over the message period~%float32 measured_battery~%float32 measured_12v~%float32 measured_5v~%~%# Current senses available on platform, in amps.~%# Averaged over the message period~%float32 drive_current~%float32 user_current~%float32 computer_current~%float32 total_current~%~%# Highest total system current peak as measured in a 1ms window.~%float32 total_current_peak~%~%# Integration of all power consumption since MCU power-on, in watt-hours.~%float64 total_power_consumed ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Status>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'hardware_id))
     8
     8
     1
     1
     1
     4
     4
     4
     4
     4
     4
     4
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Status>))
  "Converts a ROS message object to a list"
  (cl:list 'Status
    (cl:cons ':header (header msg))
    (cl:cons ':hardware_id (hardware_id msg))
    (cl:cons ':mcu_uptime (mcu_uptime msg))
    (cl:cons ':connection_uptime (connection_uptime msg))
    (cl:cons ':drivers_active (drivers_active msg))
    (cl:cons ':driver_external_stop_present (driver_external_stop_present msg))
    (cl:cons ':driver_external_stop_stopped (driver_external_stop_stopped msg))
    (cl:cons ':measured_battery (measured_battery msg))
    (cl:cons ':measured_12v (measured_12v msg))
    (cl:cons ':measured_5v (measured_5v msg))
    (cl:cons ':drive_current (drive_current msg))
    (cl:cons ':user_current (user_current msg))
    (cl:cons ':computer_current (computer_current msg))
    (cl:cons ':total_current (total_current msg))
    (cl:cons ':total_current_peak (total_current_peak msg))
    (cl:cons ':total_power_consumed (total_power_consumed msg))
))
