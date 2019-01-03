;; Auto-generated. Do not edit!


(when (boundp 'xx_msgs::Flag_xx)
  (if (not (find-package "XX_MSGS"))
    (make-package "XX_MSGS"))
  (shadow 'Flag_xx (find-package "XX_MSGS")))
(unless (find-package "XX_MSGS::FLAG_XX")
  (make-package "XX_MSGS::FLAG_XX"))
(unless (find-package "XX_MSGS::FLAG_XXREQUEST")
  (make-package "XX_MSGS::FLAG_XXREQUEST"))
(unless (find-package "XX_MSGS::FLAG_XXRESPONSE")
  (make-package "XX_MSGS::FLAG_XXRESPONSE"))

(in-package "ROS")





(defclass xx_msgs::Flag_xxRequest
  :super ros::object
  :slots (_flag ))

(defmethod xx_msgs::Flag_xxRequest
  (:init
   (&key
    ((:flag __flag) "")
    )
   (send-super :init)
   (setq _flag (string __flag))
   self)
  (:flag
   (&optional __flag)
   (if __flag (setq _flag __flag)) _flag)
  (:serialization-length
   ()
   (+
    ;; string _flag
    4 (length _flag)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _flag
       (write-long (length _flag) s) (princ _flag s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _flag
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _flag (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass xx_msgs::Flag_xxResponse
  :super ros::object
  :slots (_result ))

(defmethod xx_msgs::Flag_xxResponse
  (:init
   (&key
    ((:result __result) "")
    )
   (send-super :init)
   (setq _result (string __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; string _result
    4 (length _result)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _result
       (write-long (length _result) s) (princ _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _result
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _result (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass xx_msgs::Flag_xx
  :super ros::object
  :slots ())

(setf (get xx_msgs::Flag_xx :md5sum-) "30f07fd1b48dfe19e8223bd0e3a0cdf5")
(setf (get xx_msgs::Flag_xx :datatype-) "xx_msgs/Flag_xx")
(setf (get xx_msgs::Flag_xx :request) xx_msgs::Flag_xxRequest)
(setf (get xx_msgs::Flag_xx :response) xx_msgs::Flag_xxResponse)

(defmethod xx_msgs::Flag_xxRequest
  (:response () (instance xx_msgs::Flag_xxResponse :init)))

(setf (get xx_msgs::Flag_xxRequest :md5sum-) "30f07fd1b48dfe19e8223bd0e3a0cdf5")
(setf (get xx_msgs::Flag_xxRequest :datatype-) "xx_msgs/Flag_xxRequest")
(setf (get xx_msgs::Flag_xxRequest :definition-)
      "string flag
---
string result
")

(setf (get xx_msgs::Flag_xxResponse :md5sum-) "30f07fd1b48dfe19e8223bd0e3a0cdf5")
(setf (get xx_msgs::Flag_xxResponse :datatype-) "xx_msgs/Flag_xxResponse")
(setf (get xx_msgs::Flag_xxResponse :definition-)
      "string flag
---
string result
")



(provide :xx_msgs/Flag_xx "30f07fd1b48dfe19e8223bd0e3a0cdf5")


