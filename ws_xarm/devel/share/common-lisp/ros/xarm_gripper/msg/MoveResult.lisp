; Auto-generated. Do not edit!


(cl:in-package xarm_gripper-msg)


;//! \htmlinclude MoveResult.msg.html

(cl:defclass <MoveResult> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (err_code
    :reader err_code
    :initarg :err_code
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MoveResult (<MoveResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_gripper-msg:<MoveResult> is deprecated: use xarm_gripper-msg:MoveResult instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <MoveResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_gripper-msg:success-val is deprecated.  Use xarm_gripper-msg:success instead.")
  (success m))

(cl:ensure-generic-function 'err_code-val :lambda-list '(m))
(cl:defmethod err_code-val ((m <MoveResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_gripper-msg:err_code-val is deprecated.  Use xarm_gripper-msg:err_code instead.")
  (err_code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveResult>) ostream)
  "Serializes a message object of type '<MoveResult>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'err_code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveResult>) istream)
  "Deserializes a message object of type '<MoveResult>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'err_code) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveResult>)))
  "Returns string type for a message object of type '<MoveResult>"
  "xarm_gripper/MoveResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveResult)))
  "Returns string type for a message object of type 'MoveResult"
  "xarm_gripper/MoveResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveResult>)))
  "Returns md5sum for a message object of type '<MoveResult>"
  "bd37d58cd262d8243347be50af0c1a3f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveResult)))
  "Returns md5sum for a message object of type 'MoveResult"
  "bd37d58cd262d8243347be50af0c1a3f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveResult>)))
  "Returns full string definition for message of type '<MoveResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%bool success ~%~%int16 err_code~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveResult)))
  "Returns full string definition for message of type 'MoveResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%bool success ~%~%int16 err_code~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveResult>))
  (cl:+ 0
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveResult>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveResult
    (cl:cons ':success (success msg))
    (cl:cons ':err_code (err_code msg))
))
