;; Auto-generated. Do not edit!


(when (boundp 'vision_no_ros::panel_object)
  (if (not (find-package "VISION_NO_ROS"))
    (make-package "VISION_NO_ROS"))
  (shadow 'panel_object (find-package "VISION_NO_ROS")))
(unless (find-package "VISION_NO_ROS::PANEL_OBJECT")
  (make-package "VISION_NO_ROS::PANEL_OBJECT"))

(in-package "ROS")
;;//! \htmlinclude panel_object.msg.html


(defclass vision_no_ros::panel_object
  :super ros::object
  :slots (_id _reliability _x_pos _y_pos _z_pos _x_rot _y_rot _z_rot ))

(defmethod vision_no_ros::panel_object
  (:init
   (&key
    ((:id __id) 0)
    ((:reliability __reliability) 0)
    ((:x_pos __x_pos) 0.0)
    ((:y_pos __y_pos) 0.0)
    ((:z_pos __z_pos) 0.0)
    ((:x_rot __x_rot) 0.0)
    ((:y_rot __y_rot) 0.0)
    ((:z_rot __z_rot) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _reliability (round __reliability))
   (setq _x_pos (float __x_pos))
   (setq _y_pos (float __y_pos))
   (setq _z_pos (float __z_pos))
   (setq _x_rot (float __x_rot))
   (setq _y_rot (float __y_rot))
   (setq _z_rot (float __z_rot))
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
    ;; int8 _id
    1
    ;; int8 _reliability
    1
    ;; float32 _x_pos
    4
    ;; float32 _y_pos
    4
    ;; float32 _z_pos
    4
    ;; float32 _x_rot
    4
    ;; float32 _y_rot
    4
    ;; float32 _z_rot
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _id
       (write-byte _id s)
     ;; int8 _reliability
       (write-byte _reliability s)
     ;; float32 _x_pos
       (sys::poke _x_pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_pos
       (sys::poke _y_pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z_pos
       (sys::poke _z_pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x_rot
       (sys::poke _x_rot (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_rot
       (sys::poke _y_rot (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z_rot
       (sys::poke _z_rot (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _id 127) (setq _id (- _id 256)))
   ;; int8 _reliability
     (setq _reliability (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _reliability 127) (setq _reliability (- _reliability 256)))
   ;; float32 _x_pos
     (setq _x_pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_pos
     (setq _y_pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z_pos
     (setq _z_pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x_rot
     (setq _x_rot (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_rot
     (setq _y_rot (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z_rot
     (setq _z_rot (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get vision_no_ros::panel_object :md5sum-) "f840f14310746dd7a845272aabd4fa8c")
(setf (get vision_no_ros::panel_object :datatype-) "vision_no_ros/panel_object")
(setf (get vision_no_ros::panel_object :definition-)
      "int8 id
int8 reliability
float32 x_pos
float32 y_pos
float32 z_pos
float32 x_rot
float32 y_rot
float32 z_rot

")



(provide :vision_no_ros/panel_object "f840f14310746dd7a845272aabd4fa8c")


