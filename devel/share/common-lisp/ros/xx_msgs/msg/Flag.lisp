; Auto-generated. Do not edit!


(cl:in-package xx_msgs-msg)


;//! \htmlinclude Flag.msg.html

(cl:defclass <Flag> (roslisp-msg-protocol:ros-message)
  ((flag
    :reader flag
    :initarg :flag
    :type cl:string
    :initform ""))
)

(cl:defclass Flag (<Flag>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Flag>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Flag)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xx_msgs-msg:<Flag> is deprecated: use xx_msgs-msg:Flag instead.")))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <Flag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xx_msgs-msg:flag-val is deprecated.  Use xx_msgs-msg:flag instead.")
  (flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Flag>) ostream)
  "Serializes a message object of type '<Flag>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'flag))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'flag))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Flag>) istream)
  "Deserializes a message object of type '<Flag>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Flag>)))
  "Returns string type for a message object of type '<Flag>"
  "xx_msgs/Flag")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Flag)))
  "Returns string type for a message object of type 'Flag"
  "xx_msgs/Flag")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Flag>)))
  "Returns md5sum for a message object of type '<Flag>"
  "1042664ff3b165ca1987ef393c5212a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Flag)))
  "Returns md5sum for a message object of type 'Flag"
  "1042664ff3b165ca1987ef393c5212a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Flag>)))
  "Returns full string definition for message of type '<Flag>"
  (cl:format cl:nil "string flag~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Flag)))
  "Returns full string definition for message of type 'Flag"
  (cl:format cl:nil "string flag~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Flag>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'flag))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Flag>))
  "Converts a ROS message object to a list"
  (cl:list 'Flag
    (cl:cons ':flag (flag msg))
))
