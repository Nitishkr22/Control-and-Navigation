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
    ((:object_ID __object_ID) (make-array 0 :initial-element 0 :element-type :integer))
    ((:x __x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y __y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velx __velx) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:vely __vely) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _object_ID __object_ID)
   (setq _x __x)
   (setq _y __y)
   (setq _velx __velx)
   (setq _vely __vely)
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
    ;; int32[] _object_ID
    (* 4    (length _object_ID)) 4
    ;; float32[] _x
    (* 4    (length _x)) 4
    ;; float32[] _y
    (* 4    (length _y)) 4
    ;; float32[] _velx
    (* 4    (length _velx)) 4
    ;; float32[] _vely
    (* 4    (length _vely)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _object_ID
     (write-long (length _object_ID) s)
     (dotimes (i (length _object_ID))
       (write-long (elt _object_ID i) s)
       )
     ;; float32[] _x
     (write-long (length _x) s)
     (dotimes (i (length _x))
       (sys::poke (elt _x i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _y
     (write-long (length _y) s)
     (dotimes (i (length _y))
       (sys::poke (elt _y i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _velx
     (write-long (length _velx) s)
     (dotimes (i (length _velx))
       (sys::poke (elt _velx i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _vely
     (write-long (length _vely) s)
     (dotimes (i (length _vely))
       (sys::poke (elt _vely i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _object_ID
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _object_ID (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _object_ID i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32[] _x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _velx
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velx (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velx i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _vely
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vely (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _vely i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get conti_radar::radar_msgs :md5sum-) "b031195c2ce10d5f86075f318ebd08cc")
(setf (get conti_radar::radar_msgs :datatype-) "conti_radar/radar_msgs")
(setf (get conti_radar::radar_msgs :definition-)
      "int32[] object_ID
float32[] x
float32[] y
float32[] velx
float32[] vely

")



(provide :conti_radar/radar_msgs "b031195c2ce10d5f86075f318ebd08cc")


