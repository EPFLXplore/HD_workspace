;; Auto-generated. Do not edit!


(when (boundp 'vision_no_ros::vector_msg)
  (if (not (find-package "VISION_NO_ROS"))
    (make-package "VISION_NO_ROS"))
  (shadow 'vector_msg (find-package "VISION_NO_ROS")))
(unless (find-package "VISION_NO_ROS::VECTOR_MSG")
  (make-package "VISION_NO_ROS::VECTOR_MSG"))

(in-package "ROS")
;;//! \htmlinclude vector_msg.msg.html


(defclass vision_no_ros::vector_msg
  :super ros::object
  :slots (_id _reliability _x_pos _y_pos _z_pos _x_rot _y_rot _z_rot ))

(defmethod vision_no_ros::vector_msg
  (:init
   (&key
    ((:id __id) 0)
    ((:reliability __reliability) 0)
    ((:x_pos __x_pos) 0)
    ((:y_pos __y_pos) 0)
    ((:z_pos __z_pos) 0)
    ((:x_rot __x_rot) 0)
    ((:y_rot __y_rot) 0)
    ((:z_rot __z_rot) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _reliability (round __reliability))
   (setq _x_pos (round __x_pos))
   (setq _y_pos (round __y_pos))
   (setq _z_pos (round __z_pos))
   (setq _x_rot (round __x_rot))
   (setq _y_rot (round __y_rot))
   (setq _z_rot (round __z_rot))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:reliability
   (&optional __reliability)
   (if __reliability (setq _reliability __reliability)) _reliability)
  (:x_pos
   (&optional __x_pos)
   (if __x_pos (setq _x_pos __x_pos)) _x_pos)
  (:y_pos
   (&optional __y_pos)
   (if __y_pos (setq _y_pos __y_pos)) _y_pos)
  (:z_pos
   (&optional __z_pos)
   (if __z_pos (setq _z_pos __z_pos)) _z_pos)
  (:x_rot
   (&optional __x_rot)
   (if __x_rot (setq _x_rot __x_rot)) _x_rot)
  (:y_rot
   (&optional __y_rot)
   (if __y_rot (setq _y_rot __y_rot)) _y_rot)
  (:z_rot
   (&optional __z_rot)
   (if __z_rot (setq _z_rot __z_rot)) _z_rot)
  (:serialization-length
   ()
   (+
    ;; int16 _id
    2
    ;; int8 _reliability
    1
    ;; int16 _x_pos
    2
    ;; int16 _y_pos
    2
    ;; int16 _z_pos
    2
    ;; int16 _x_rot
    2
    ;; int16 _y_rot
    2
    ;; int16 _z_rot
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _id
       (write-word _id s)
     ;; int8 _reliability
       (write-byte _reliability s)
     ;; int16 _x_pos
       (write-word _x_pos s)
     ;; int16 _y_pos
       (write-word _y_pos s)
     ;; int16 _z_pos
       (write-word _z_pos s)
     ;; int16 _x_rot
       (write-word _x_rot s)
     ;; int16 _y_rot
       (write-word _y_rot s)
     ;; int16 _z_rot
       (write-word _z_rot s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _id
     (setq _id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int8 _reliability
     (setq _reliability (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _reliability 127) (setq _reliability (- _reliability 256)))
   ;; int16 _x_pos
     (setq _x_pos (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _y_pos
     (setq _y_pos (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _z_pos
     (setq _z_pos (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _x_rot
     (setq _x_rot (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _y_rot
     (setq _y_rot (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _z_rot
     (setq _z_rot (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get vision_no_ros::vector_msg :md5sum-) "5293413431842928eb9de0ce34594eab")
(setf (get vision_no_ros::vector_msg :datatype-) "vision_no_ros/vector_msg")
(setf (get vision_no_ros::vector_msg :definition-)
      "int16 id
int8 reliability
int16 x_pos
int16 y_pos
int16 z_pos
int16 x_rot
int16 y_rot
int16 z_rot

")



(provide :vision_no_ros/vector_msg "5293413431842928eb9de0ce34594eab")


