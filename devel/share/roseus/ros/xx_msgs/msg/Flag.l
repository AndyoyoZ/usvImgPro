;; Auto-generated. Do not edit!


(when (boundp 'xx_msgs::Flag)
  (if (not (find-package "XX_MSGS"))
    (make-package "XX_MSGS"))
  (shadow 'Flag (find-package "XX_MSGS")))
(unless (find-package "XX_MSGS::FLAG")
  (make-package "XX_MSGS::FLAG"))

(in-package "ROS")
;;//! \htmlinclude Flag.msg.html


(defclass xx_msgs::Flag
  :super ros::object
  :slots (_flag ))

(defmethod xx_msgs::Flag
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

(setf (get xx_msgs::Flag :md5sum-) "1042664ff3b165ca1987ef393c5212a5")
(setf (get xx_msgs::Flag :datatype-) "xx_msgs/Flag")
(setf (get xx_msgs::Flag :definition-)
      "string flag
")



(provide :xx_msgs/Flag "1042664ff3b165ca1987ef393c5212a5")


