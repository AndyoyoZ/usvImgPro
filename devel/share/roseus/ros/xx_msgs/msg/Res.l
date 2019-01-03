;; Auto-generated. Do not edit!


(when (boundp 'xx_msgs::Res)
  (if (not (find-package "XX_MSGS"))
    (make-package "XX_MSGS"))
  (shadow 'Res (find-package "XX_MSGS")))
(unless (find-package "XX_MSGS::RES")
  (make-package "XX_MSGS::RES"))

(in-package "ROS")
;;//! \htmlinclude Res.msg.html


(defclass xx_msgs::Res
  :super ros::object
  :slots (_result ))

(defmethod xx_msgs::Res
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

(setf (get xx_msgs::Res :md5sum-) "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(setf (get xx_msgs::Res :datatype-) "xx_msgs/Res")
(setf (get xx_msgs::Res :definition-)
      "string result
")



(provide :xx_msgs/Res "c22f2a1ed8654a0b365f1bb3f7ff2c0f")


