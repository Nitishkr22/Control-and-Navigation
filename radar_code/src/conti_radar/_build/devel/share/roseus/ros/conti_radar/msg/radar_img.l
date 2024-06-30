;; Auto-generated. Do not edit!


(when (boundp 'conti_radar::radar_img)
  (if (not (find-package "CONTI_RADAR"))
    (make-package "CONTI_RADAR"))
  (shadow 'radar_img (find-package "CONTI_RADAR")))
(unless (find-package "CONTI_RADAR::RADAR_IMG")
  (make-package "CONTI_RADAR::RADAR_IMG"))

(in-package "ROS")
;;//! \htmlinclude radar_img.msg.html


(defclass conti_radar::radar_img
  :super ros::object
  :slots (_id _range _azimuth _VrelRad _RCS _rangevar _vrelradvar _azimuthvar _SNR ))

(defmethod conti_radar::radar_img
  (:init
   (&key
    ((:id __id) 0)
    ((:range __range) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:azimuth __azimuth) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:VrelRad __VrelRad) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:RCS __RCS) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:rangevar __rangevar) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:vrelradvar __vrelradvar) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:azimuthvar __azimuthvar) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:SNR __SNR) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _range __range)
   (setq _azimuth __azimuth)
   (setq _VrelRad __VrelRad)
   (setq _RCS __RCS)
   (setq _rangevar __rangevar)
   (setq _vrelradvar __vrelradvar)
   (setq _azimuthvar __azimuthvar)
   (setq _SNR __SNR)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:azimuth
   (&optional __azimuth)
   (if __azimuth (setq _azimuth __azimuth)) _azimuth)
  (:VrelRad
   (&optional __VrelRad)
   (if __VrelRad (setq _VrelRad __VrelRad)) _VrelRad)
  (:RCS
   (&optional __RCS)
   (if __RCS (setq _RCS __RCS)) _RCS)
  (:rangevar
   (&optional __rangevar)
   (if __rangevar (setq _rangevar __rangevar)) _rangevar)
  (:vrelradvar
   (&optional __vrelradvar)
   (if __vrelradvar (setq _vrelradvar __vrelradvar)) _vrelradvar)
  (:azimuthvar
   (&optional __azimuthvar)
   (if __azimuthvar (setq _azimuthvar __azimuthvar)) _azimuthvar)
  (:SNR
   (&optional __SNR)
   (if __SNR (setq _SNR __SNR)) _SNR)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; float32[] _range
    (* 4    (length _range)) 4
    ;; float32[] _azimuth
    (* 4    (length _azimuth)) 4
    ;; float32[] _VrelRad
    (* 4    (length _VrelRad)) 4
    ;; float32[] _RCS
    (* 4    (length _RCS)) 4
    ;; float32[] _rangevar
    (* 4    (length _rangevar)) 4
    ;; float32[] _vrelradvar
    (* 4    (length _vrelradvar)) 4
    ;; float32[] _azimuthvar
    (* 4    (length _azimuthvar)) 4
    ;; float32[] _SNR
    (* 4    (length _SNR)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; float32[] _range
     (write-long (length _range) s)
     (dotimes (i (length _range))
       (sys::poke (elt _range i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _azimuth
     (write-long (length _azimuth) s)
     (dotimes (i (length _azimuth))
       (sys::poke (elt _azimuth i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _VrelRad
     (write-long (length _VrelRad) s)
     (dotimes (i (length _VrelRad))
       (sys::poke (elt _VrelRad i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _RCS
     (write-long (length _RCS) s)
     (dotimes (i (length _RCS))
       (sys::poke (elt _RCS i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _rangevar
     (write-long (length _rangevar) s)
     (dotimes (i (length _rangevar))
       (sys::poke (elt _rangevar i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _vrelradvar
     (write-long (length _vrelradvar) s)
     (dotimes (i (length _vrelradvar))
       (sys::poke (elt _vrelradvar i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _azimuthvar
     (write-long (length _azimuthvar) s)
     (dotimes (i (length _azimuthvar))
       (sys::poke (elt _azimuthvar i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _SNR
     (write-long (length _SNR) s)
     (dotimes (i (length _SNR))
       (sys::poke (elt _SNR i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _range
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _range (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _range i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _azimuth
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _azimuth (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _azimuth i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _VrelRad
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _VrelRad (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _VrelRad i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _RCS
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _RCS (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _RCS i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _rangevar
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rangevar (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _rangevar i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _vrelradvar
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vrelradvar (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _vrelradvar i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _azimuthvar
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _azimuthvar (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _azimuthvar i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _SNR
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _SNR (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _SNR i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get conti_radar::radar_img :md5sum-) "4bee43583744ef6616006334aa5e5b2e")
(setf (get conti_radar::radar_img :datatype-) "conti_radar/radar_img")
(setf (get conti_radar::radar_img :definition-)
      "int32 id
float32[] range
float32[] azimuth
float32[] VrelRad
float32[] RCS
float32[] rangevar
float32[] vrelradvar
float32[] azimuthvar
float32[] SNR
")



(provide :conti_radar/radar_img "4bee43583744ef6616006334aa5e5b2e")


