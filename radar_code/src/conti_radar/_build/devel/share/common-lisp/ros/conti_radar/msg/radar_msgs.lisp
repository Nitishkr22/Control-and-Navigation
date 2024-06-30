; Auto-generated. Do not edit!


(cl:in-package conti_radar-msg)


;//! \htmlinclude radar_msgs.msg.html

(cl:defclass <radar_msgs> (roslisp-msg-protocol:ros-message)
  ((object_ID
    :reader object_ID
    :initarg :object_ID
    :type cl:integer
    :initform 0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (velx
    :reader velx
    :initarg :velx
    :type cl:float
    :initform 0.0)
   (vely
    :reader vely
    :initarg :vely
    :type cl:float
    :initform 0.0))
)

(cl:defclass radar_msgs (<radar_msgs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <radar_msgs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'radar_msgs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name conti_radar-msg:<radar_msgs> is deprecated: use conti_radar-msg:radar_msgs instead.")))

(cl:ensure-generic-function 'object_ID-val :lambda-list '(m))
(cl:defmethod object_ID-val ((m <radar_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader conti_radar-msg:object_ID-val is deprecated.  Use conti_radar-msg:object_ID instead.")
  (object_ID m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <radar_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader conti_radar-msg:x-val is deprecated.  Use conti_radar-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <radar_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader conti_radar-msg:y-val is deprecated.  Use conti_radar-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'velx-val :lambda-list '(m))
(cl:defmethod velx-val ((m <radar_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader conti_radar-msg:velx-val is deprecated.  Use conti_radar-msg:velx instead.")
  (velx m))

(cl:ensure-generic-function 'vely-val :lambda-list '(m))
(cl:defmethod vely-val ((m <radar_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader conti_radar-msg:vely-val is deprecated.  Use conti_radar-msg:vely instead.")
  (vely m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <radar_msgs>) ostream)
  "Serializes a message object of type '<radar_msgs>"
  (cl:let* ((signed (cl:slot-value msg 'object_ID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vely))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <radar_msgs>) istream)
  "Deserializes a message object of type '<radar_msgs>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object_ID) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vely) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<radar_msgs>)))
  "Returns string type for a message object of type '<radar_msgs>"
  "conti_radar/radar_msgs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'radar_msgs)))
  "Returns string type for a message object of type 'radar_msgs"
  "conti_radar/radar_msgs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<radar_msgs>)))
  "Returns md5sum for a message object of type '<radar_msgs>"
  "877f9ca9fe90682b5f51d194a8ed2ce9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'radar_msgs)))
  "Returns md5sum for a message object of type 'radar_msgs"
  "877f9ca9fe90682b5f51d194a8ed2ce9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<radar_msgs>)))
  "Returns full string definition for message of type '<radar_msgs>"
  (cl:format cl:nil "int32 object_ID~%float32 x~%float32 y~%float32 velx~%float32 vely~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'radar_msgs)))
  "Returns full string definition for message of type 'radar_msgs"
  (cl:format cl:nil "int32 object_ID~%float32 x~%float32 y~%float32 velx~%float32 vely~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <radar_msgs>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <radar_msgs>))
  "Converts a ROS message object to a list"
  (cl:list 'radar_msgs
    (cl:cons ':object_ID (object_ID msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':velx (velx msg))
    (cl:cons ':vely (vely msg))
))
