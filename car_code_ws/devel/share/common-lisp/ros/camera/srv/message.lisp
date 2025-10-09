; Auto-generated. Do not edit!


(cl:in-package camera-srv)


;//! \htmlinclude message-request.msg.html

(cl:defclass <message-request> (roslisp-msg-protocol:ros-message)
  ((if_request
    :reader if_request
    :initarg :if_request
    :type cl:integer
    :initform 0))
)

(cl:defclass message-request (<message-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <message-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'message-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera-srv:<message-request> is deprecated: use camera-srv:message-request instead.")))

(cl:ensure-generic-function 'if_request-val :lambda-list '(m))
(cl:defmethod if_request-val ((m <message-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-srv:if_request-val is deprecated.  Use camera-srv:if_request instead.")
  (if_request m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <message-request>) ostream)
  "Serializes a message object of type '<message-request>"
  (cl:let* ((signed (cl:slot-value msg 'if_request)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <message-request>) istream)
  "Deserializes a message object of type '<message-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'if_request) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<message-request>)))
  "Returns string type for a service object of type '<message-request>"
  "camera/messageRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'message-request)))
  "Returns string type for a service object of type 'message-request"
  "camera/messageRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<message-request>)))
  "Returns md5sum for a message object of type '<message-request>"
  "b9d1c9fee0054923e6d8acbb284d7b0b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'message-request)))
  "Returns md5sum for a message object of type 'message-request"
  "b9d1c9fee0054923e6d8acbb284d7b0b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<message-request>)))
  "Returns full string definition for message of type '<message-request>"
  (cl:format cl:nil "# 客户端请求时发送的两个数字~%int32 if_request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'message-request)))
  "Returns full string definition for message of type 'message-request"
  (cl:format cl:nil "# 客户端请求时发送的两个数字~%int32 if_request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <message-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <message-request>))
  "Converts a ROS message object to a list"
  (cl:list 'message-request
    (cl:cons ':if_request (if_request msg))
))
;//! \htmlinclude message-response.msg.html

(cl:defclass <message-response> (roslisp-msg-protocol:ros-message)
  ((x_p
    :reader x_p
    :initarg :x_p
    :type cl:float
    :initform 0.0)
   (y_q
    :reader y_q
    :initarg :y_q
    :type cl:float
    :initform 0.0)
   (depth
    :reader depth
    :initarg :depth
    :type cl:float
    :initform 0.0))
)

(cl:defclass message-response (<message-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <message-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'message-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera-srv:<message-response> is deprecated: use camera-srv:message-response instead.")))

(cl:ensure-generic-function 'x_p-val :lambda-list '(m))
(cl:defmethod x_p-val ((m <message-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-srv:x_p-val is deprecated.  Use camera-srv:x_p instead.")
  (x_p m))

(cl:ensure-generic-function 'y_q-val :lambda-list '(m))
(cl:defmethod y_q-val ((m <message-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-srv:y_q-val is deprecated.  Use camera-srv:y_q instead.")
  (y_q m))

(cl:ensure-generic-function 'depth-val :lambda-list '(m))
(cl:defmethod depth-val ((m <message-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-srv:depth-val is deprecated.  Use camera-srv:depth instead.")
  (depth m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <message-response>) ostream)
  "Serializes a message object of type '<message-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x_p))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y_q))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <message-response>) istream)
  "Deserializes a message object of type '<message-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_p) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_q) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'depth) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<message-response>)))
  "Returns string type for a service object of type '<message-response>"
  "camera/messageResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'message-response)))
  "Returns string type for a service object of type 'message-response"
  "camera/messageResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<message-response>)))
  "Returns md5sum for a message object of type '<message-response>"
  "b9d1c9fee0054923e6d8acbb284d7b0b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'message-response)))
  "Returns md5sum for a message object of type 'message-response"
  "b9d1c9fee0054923e6d8acbb284d7b0b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<message-response>)))
  "Returns full string definition for message of type '<message-response>"
  (cl:format cl:nil "# 服务器响应发送的数据~%float64 x_p~%float64 y_q~%float64 depth~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'message-response)))
  "Returns full string definition for message of type 'message-response"
  (cl:format cl:nil "# 服务器响应发送的数据~%float64 x_p~%float64 y_q~%float64 depth~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <message-response>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <message-response>))
  "Converts a ROS message object to a list"
  (cl:list 'message-response
    (cl:cons ':x_p (x_p msg))
    (cl:cons ':y_q (y_q msg))
    (cl:cons ':depth (depth msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'message)))
  'message-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'message)))
  'message-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'message)))
  "Returns string type for a service object of type '<message>"
  "camera/message")