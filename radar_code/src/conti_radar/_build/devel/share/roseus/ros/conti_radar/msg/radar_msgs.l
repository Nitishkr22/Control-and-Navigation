;; Auto-generated. Do not edit!


(when (boundp 'conti_radar::radar_msgs)
  (if (not (find-package "CONTI_RADAR"))
    (make-package "CONTI_RADAR"))
  (shadow 'radar_msgs (find-package "CONTI_RADAR")))
(unless (find-package "CONTI_RADAR::RADAR_MSGS")
  (make-package "CONTI_RADAR::RADAR_MSGS"))

(in-package "ROS")
;;//! \htmlinclude radar_msgs.msg.html


(defclass conti_radar::radar_msgs
  :super ros::object
  :slots (_object_ID _x _y _velx _vely ))

(defmethod conti_radar::radar_msgs
  (:init
   (&key
    ((:object_ID __object_ID) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:velx __velx) 0.0)
    ((:vely __vely) 0.0)
    )
   (send-super :init)
   (setq _object_ID (round __object_ID))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _velx (float __velx))
   (setq _vely (float __vely))
   self)
  (:object_ID
   (&optional __object_ID)
   (if __object_ID (setq _object_ID __object_ID)) _object_ID)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:velx
   (&optional __velx)
   (if __velx (setq _velx __velx)) _velx)
  (:vely
   (&optional __vely)
   (if __vely (setq _vely __vely)) _vely)
  (:serialization-length
   ()
   (+
    ;; int32 _object_ID
    4
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _velx
    4
    ;; float32 _vely
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _object_ID
       (write-long _object_ID s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _velx
       (sys::poke _velx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vely
       (sys::poke _vely (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _object_ID
     (setq _object_ID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _velx
     (setq _velx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vely
     (setq _vely (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get conti_radar::radar_msgs :md5sum-) "877f9ca9fe90682b5f51d194a8ed2ce9")
(setf (get conti_radar::radar_msgs :datatype-) "conti_radar/radar_msgs")
(setf (get conti_radar::radar_msgs :definition-)
      "int32 object_ID
float32 x
float32 y
float32 velx
float32 vely

")



(provide :conti_radar/radar_msgs "877f9ca9fe90682b5f51d194a8ed2ce9")


