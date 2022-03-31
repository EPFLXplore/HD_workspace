; Auto-generated. Do not edit!


(cl:in-package vision_no_ros-msg)


;//! \htmlinclude vector_msg.msg.html

(cl:defclass <vector_msg> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (reliability
    :reader reliability
    :initarg :reliability
    :type cl:fixnum
    :initform 0)
   (x_pos
    :reader x_pos
    :initarg :x_pos
    :type cl:fixnum
    :initform 0)
   (y_pos
    :reader y_pos
    :initarg :y_pos
    :type cl:fixnum
    :initform 0)
   (z_pos
    :reader z_pos
    :initarg :z_pos
    :type cl:fixnum
    :initform 0)
   (x_rot
    :reader x_rot
    :initarg :x_rot
    :type cl:fixnum
    :initform 0)
   (y_rot
    :reader y_rot
    :initarg :y_rot
    :type cl:fixnum
    :initform 0)
   (z_rot
    :reader z_rot
    :initarg :z_rot
    :type cl:fixnum
    :initform 0))
)

(cl:defclass vector_msg (<vector_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vector_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vector_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision_no_ros-msg:<vector_msg> is deprecated: use vision_no_ros-msg:vector_msg instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <vector_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_no_ros-msg:id-val is deprecated.  Use vision_no_ros-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'reliability-val :lambda-list '(m))
(cl:defmethod reliability-val ((m <vector_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_no_ros-msg:reliability-val is deprecated.  Use vision_no_ros-msg:reliability instead.")
  (reliability m))

(cl:ensure-generic-function 'x_pos-val :lambda-list '(m))
(cl:defmethod x_pos-val ((m <vector_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_no_ros-msg:x_pos-val is deprecated.  Use vision_no_ros-msg:x_pos instead.")
  (x_pos m))

(cl:ensure-generic-function 'y_pos-val :lambda-list '(m))
(cl:defmethod y_pos-val ((m <vector_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_no_ros-msg:y_pos-val is deprecated.  Use vision_no_ros-msg:y_pos instead.")
  (y_pos m))

(cl:ensure-generic-function 'z_pos-val :lambda-list '(m))
(cl:defmethod z_pos-val ((m <vector_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_no_ros-msg:z_pos-val is deprecated.  Use vision_no_ros-msg:z_pos instead.")
  (z_pos m))

(cl:ensure-generic-function 'x_rot-val :lambda-list '(m))
(cl:defmethod x_rot-val ((m <vector_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_no_ros-msg:x_rot-val is deprecated.  Use vision_no_ros-msg:x_rot instead.")
  (x_rot m))

(cl:ensure-generic-function 'y_rot-val :lambda-list '(m))
(cl:defmethod y_rot-val ((m <vector_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_no_ros-msg:y_rot-val is deprecated.  Use vision_no_ros-msg:y_rot instead.")
  (y_rot m))

(cl:ensure-generic-function 'z_rot-val :lambda-list '(m))
(cl:defmethod z_rot-val ((m <vector_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_no_ros-msg:z_rot-val is deprecated.  Use vision_no_ros-msg:z_rot instead.")
  (z_rot m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vector_msg>) ostream)
  "Serializes a message object of type '<vector_msg>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'reliability)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'x_pos)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y_pos)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'z_pos)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'x_rot)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y_rot)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'z_rot)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vector_msg>) istream)
  "Deserializes a message object of type '<vector_msg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reliability) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x_pos) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y_pos) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'z_pos) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x_rot) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y_rot) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'z_rot) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vector_msg>)))
  "Returns string type for a message object of type '<vector_msg>"
  "vision_no_ros/vector_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vector_msg)))
  "Returns string type for a message object of type 'vector_msg"
  "vision_no_ros/vector_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vector_msg>)))
  "Returns md5sum for a message object of type '<vector_msg>"
  "5293413431842928eb9de0ce34594eab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vector_msg)))
  "Returns md5sum for a message object of type 'vector_msg"
  "5293413431842928eb9de0ce34594eab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vector_msg>)))
  "Returns full string definition for message of type '<vector_msg>"
  (cl:format cl:nil "int16 id~%int8 reliability~%int16 x_pos~%int16 y_pos~%int16 z_pos~%int16 x_rot~%int16 y_rot~%int16 z_rot~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vector_msg)))
  "Returns full string definition for message of type 'vector_msg"
  (cl:format cl:nil "int16 id~%int8 reliability~%int16 x_pos~%int16 y_pos~%int16 z_pos~%int16 x_rot~%int16 y_rot~%int16 z_rot~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vector_msg>))
  (cl:+ 0
     2
     1
     2
     2
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vector_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'vector_msg
    (cl:cons ':id (id msg))
    (cl:cons ':reliability (reliability msg))
    (cl:cons ':x_pos (x_pos msg))
    (cl:cons ':y_pos (y_pos msg))
    (cl:cons ':z_pos (z_pos msg))
    (cl:cons ':x_rot (x_rot msg))
    (cl:cons ':y_rot (y_rot msg))
    (cl:cons ':z_rot (z_rot msg))
))
