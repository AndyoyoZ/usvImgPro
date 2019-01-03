; Auto-generated. Do not edit!


(cl:in-package xx_msgs-srv)


;//! \htmlinclude Flag_xx-request.msg.html

(cl:defclass <Flag_xx-request> (roslisp-msg-protocol:ros-message)
  ((flag
    :reader flag
    :initarg :flag
    :type cl:string
    :initform ""))
)

(cl:defclass Flag_xx-request (<Flag_xx-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Flag_xx-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Flag_xx-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xx_msgs-srv:<Flag_xx-request> is deprecated: use xx_msgs-srv:Flag_xx-request instead.")))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <Flag_xx-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xx_msgs-srv:flag-val is deprecated.  Use xx_msgs-srv:flag instead.")
  (flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Flag_xx-request>) ostream)
  "Serializes a message object of type '<Flag_xx-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'flag))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'flag))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Flag_xx-request>) istream)
  "Deserializes a message object of type '<Flag_xx-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'flag) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'flag) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Flag_xx-request>)))
  "Returns string type for a service object of type '<Flag_xx-request>"
  "xx_msgs/Flag_xxRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Flag_xx-request)))
  "Returns string type for a service object of type 'Flag_xx-request"
  "xx_msgs/Flag_xxRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Flag_xx-request>)))
  "Returns md5sum for a message object of type '<Flag_xx-request>"
  "30f07fd1b48dfe19e8223bd0e3a0cdf5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Flag_xx-request)))
  "Returns md5sum for a message object of type 'Flag_xx-request"
  "30f07fd1b48dfe19e8223bd0e3a0cdf5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Flag_xx-request>)))
  "Returns full string definition for message of type '<Flag_xx-request>"
  (cl:format cl:nil "string flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Flag_xx-request)))
  "Returns full string definition for message of type 'Flag_xx-request"
  (cl:format cl:nil "string flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Flag_xx-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'flag))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Flag_xx-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Flag_xx-request
    (cl:cons ':flag (flag msg))
))
;//! \htmlinclude Flag_xx-response.msg.html

(cl:defclass <Flag_xx-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass Flag_xx-response (<Flag_xx-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Flag_xx-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Flag_xx-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xx_msgs-srv:<Flag_xx-response> is deprecated: use xx_msgs-srv:Flag_xx-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <Flag_xx-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xx_msgs-srv:result-val is deprecated.  Use xx_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Flag_xx-response>) ostream)
  "Serializes a message object of type '<Flag_xx-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Flag_xx-response>) istream)
  "Deserializes a message object of type '<Flag_xx-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Flag_xx-response>)))
  "Returns string type for a service object of type '<Flag_xx-response>"
  "xx_msgs/Flag_xxResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Flag_xx-response)))
  "Returns string type for a service object of type 'Flag_xx-response"
  "xx_msgs/Flag_xxResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Flag_xx-response>)))
  "Returns md5sum for a message object of type '<Flag_xx-response>"
  "30f07fd1b48dfe19e8223bd0e3a0cdf5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Flag_xx-response)))
  "Returns md5sum for a message object of type 'Flag_xx-response"
  "30f07fd1b48dfe19e8223bd0e3a0cdf5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Flag_xx-response>)))
  "Returns full string definition for message of type '<Flag_xx-response>"
  (cl:format cl:nil "string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Flag_xx-response)))
  "Returns full string definition for message of type 'Flag_xx-response"
  (cl:format cl:nil "string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Flag_xx-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Flag_xx-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Flag_xx-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Flag_xx)))
  'Flag_xx-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Flag_xx)))
  'Flag_xx-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Flag_xx)))
  "Returns string type for a service object of type '<Flag_xx>"
  "xx_msgs/Flag_xx")