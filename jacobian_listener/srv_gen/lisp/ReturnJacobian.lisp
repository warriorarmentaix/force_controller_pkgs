; Auto-generated. Do not edit!


(cl:in-package jacobian_listener-srv)


;//! \htmlinclude ReturnJacobian-request.msg.html

(cl:defclass <ReturnJacobian-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ReturnJacobian-request (<ReturnJacobian-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReturnJacobian-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReturnJacobian-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jacobian_listener-srv:<ReturnJacobian-request> is deprecated: use jacobian_listener-srv:ReturnJacobian-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReturnJacobian-request>) ostream)
  "Serializes a message object of type '<ReturnJacobian-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReturnJacobian-request>) istream)
  "Deserializes a message object of type '<ReturnJacobian-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReturnJacobian-request>)))
  "Returns string type for a service object of type '<ReturnJacobian-request>"
  "jacobian_listener/ReturnJacobianRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReturnJacobian-request)))
  "Returns string type for a service object of type 'ReturnJacobian-request"
  "jacobian_listener/ReturnJacobianRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReturnJacobian-request>)))
  "Returns md5sum for a message object of type '<ReturnJacobian-request>"
  "dd1586518916317e3922f53e1c2cb523")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReturnJacobian-request)))
  "Returns md5sum for a message object of type 'ReturnJacobian-request"
  "dd1586518916317e3922f53e1c2cb523")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReturnJacobian-request>)))
  "Returns full string definition for message of type '<ReturnJacobian-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReturnJacobian-request)))
  "Returns full string definition for message of type 'ReturnJacobian-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReturnJacobian-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReturnJacobian-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReturnJacobian-request
))
;//! \htmlinclude ReturnJacobian-response.msg.html

(cl:defclass <ReturnJacobian-response> (roslisp-msg-protocol:ros-message)
  ((jacobian
    :reader jacobian
    :initarg :jacobian
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ReturnJacobian-response (<ReturnJacobian-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReturnJacobian-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReturnJacobian-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jacobian_listener-srv:<ReturnJacobian-response> is deprecated: use jacobian_listener-srv:ReturnJacobian-response instead.")))

(cl:ensure-generic-function 'jacobian-val :lambda-list '(m))
(cl:defmethod jacobian-val ((m <ReturnJacobian-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jacobian_listener-srv:jacobian-val is deprecated.  Use jacobian_listener-srv:jacobian instead.")
  (jacobian m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReturnJacobian-response>) ostream)
  "Serializes a message object of type '<ReturnJacobian-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'jacobian))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'jacobian))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReturnJacobian-response>) istream)
  "Deserializes a message object of type '<ReturnJacobian-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'jacobian) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'jacobian)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReturnJacobian-response>)))
  "Returns string type for a service object of type '<ReturnJacobian-response>"
  "jacobian_listener/ReturnJacobianResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReturnJacobian-response)))
  "Returns string type for a service object of type 'ReturnJacobian-response"
  "jacobian_listener/ReturnJacobianResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReturnJacobian-response>)))
  "Returns md5sum for a message object of type '<ReturnJacobian-response>"
  "dd1586518916317e3922f53e1c2cb523")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReturnJacobian-response)))
  "Returns md5sum for a message object of type 'ReturnJacobian-response"
  "dd1586518916317e3922f53e1c2cb523")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReturnJacobian-response>)))
  "Returns full string definition for message of type '<ReturnJacobian-response>"
  (cl:format cl:nil "float64[] jacobian~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReturnJacobian-response)))
  "Returns full string definition for message of type 'ReturnJacobian-response"
  (cl:format cl:nil "float64[] jacobian~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReturnJacobian-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'jacobian) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReturnJacobian-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReturnJacobian-response
    (cl:cons ':jacobian (jacobian msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReturnJacobian)))
  'ReturnJacobian-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReturnJacobian)))
  'ReturnJacobian-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReturnJacobian)))
  "Returns string type for a service object of type '<ReturnJacobian>"
  "jacobian_listener/ReturnJacobian")