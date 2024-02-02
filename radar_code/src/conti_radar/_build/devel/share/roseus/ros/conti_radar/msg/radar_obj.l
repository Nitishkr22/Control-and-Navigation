;; Auto-generated. Do not edit!


(when (boundp 'conti_radar::radar_obj)
  (if (not (find-package "CONTI_RADAR"))
    (make-package "CONTI_RADAR"))
  (shadow 'radar_obj (find-package "CONTI_RADAR")))
(unless (find-package "CONTI_RADAR::RADAR_OBJ")
  (make-package "CONTI_RADAR::RADAR_OBJ"))

(in-package "ROS")
;;//! \htmlinclude radar_obj.msg.html


(defclass conti_radar::radar_obj
  :super ros::object
  :slots (_id _f_DistX _f_DistY _f_VrelX _f_VrelY _f_ArelX _f_ArelY _f_DistXStd _f_DistYStd _f_VrelXStd _f_VrelYStd _f_ArelXStd _f_ArelYStd _f_LDeltaX_left _f_LDeltaX_mid _f_LDeltaX_right _f_LDeltaY_left _f_LDeltaY_mid _f_LDeltaY_right _f_RCS _f_ObjectScore _f_VabsX _f_VabsY _f_AabsX _f_AabsY _f_VabsXStd _f_VabsYStd _f_AabsXStd _f_AabsYStd _f_Orientation _f_OrientationStd _f_DistZ _f_DistZStd ))

(defmethod conti_radar::radar_obj
  (:init
   (&key
    ((:id __id) 0)
    ((:f_DistX __f_DistX) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_DistY __f_DistY) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_VrelX __f_VrelX) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_VrelY __f_VrelY) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_ArelX __f_ArelX) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_ArelY __f_ArelY) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_DistXStd __f_DistXStd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_DistYStd __f_DistYStd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_VrelXStd __f_VrelXStd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_VrelYStd __f_VrelYStd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_ArelXStd __f_ArelXStd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_ArelYStd __f_ArelYStd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_LDeltaX_left __f_LDeltaX_left) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_LDeltaX_mid __f_LDeltaX_mid) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_LDeltaX_right __f_LDeltaX_right) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_LDeltaY_left __f_LDeltaY_left) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_LDeltaY_mid __f_LDeltaY_mid) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_LDeltaY_right __f_LDeltaY_right) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_RCS __f_RCS) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_ObjectScore __f_ObjectScore) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_VabsX __f_VabsX) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_VabsY __f_VabsY) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_AabsX __f_AabsX) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_AabsY __f_AabsY) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_VabsXStd __f_VabsXStd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_VabsYStd __f_VabsYStd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_AabsXStd __f_AabsXStd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_AabsYStd __f_AabsYStd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_Orientation __f_Orientation) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_OrientationStd __f_OrientationStd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_DistZ __f_DistZ) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_DistZStd __f_DistZStd) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _f_DistX __f_DistX)
   (setq _f_DistY __f_DistY)
   (setq _f_VrelX __f_VrelX)
   (setq _f_VrelY __f_VrelY)
   (setq _f_ArelX __f_ArelX)
   (setq _f_ArelY __f_ArelY)
   (setq _f_DistXStd __f_DistXStd)
   (setq _f_DistYStd __f_DistYStd)
   (setq _f_VrelXStd __f_VrelXStd)
   (setq _f_VrelYStd __f_VrelYStd)
   (setq _f_ArelXStd __f_ArelXStd)
   (setq _f_ArelYStd __f_ArelYStd)
   (setq _f_LDeltaX_left __f_LDeltaX_left)
   (setq _f_LDeltaX_mid __f_LDeltaX_mid)
   (setq _f_LDeltaX_right __f_LDeltaX_right)
   (setq _f_LDeltaY_left __f_LDeltaY_left)
   (setq _f_LDeltaY_mid __f_LDeltaY_mid)
   (setq _f_LDeltaY_right __f_LDeltaY_right)
   (setq _f_RCS __f_RCS)
   (setq _f_ObjectScore __f_ObjectScore)
   (setq _f_VabsX __f_VabsX)
   (setq _f_VabsY __f_VabsY)
   (setq _f_AabsX __f_AabsX)
   (setq _f_AabsY __f_AabsY)
   (setq _f_VabsXStd __f_VabsXStd)
   (setq _f_VabsYStd __f_VabsYStd)
   (setq _f_AabsXStd __f_AabsXStd)
   (setq _f_AabsYStd __f_AabsYStd)
   (setq _f_Orientation __f_Orientation)
   (setq _f_OrientationStd __f_OrientationStd)
   (setq _f_DistZ __f_DistZ)
   (setq _f_DistZStd __f_DistZStd)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:f_DistX
   (&optional __f_DistX)
   (if __f_DistX (setq _f_DistX __f_DistX)) _f_DistX)
  (:f_DistY
   (&optional __f_DistY)
   (if __f_DistY (setq _f_DistY __f_DistY)) _f_DistY)
  (:f_VrelX
   (&optional __f_VrelX)
   (if __f_VrelX (setq _f_VrelX __f_VrelX)) _f_VrelX)
  (:f_VrelY
   (&optional __f_VrelY)
   (if __f_VrelY (setq _f_VrelY __f_VrelY)) _f_VrelY)
  (:f_ArelX
   (&optional __f_ArelX)
   (if __f_ArelX (setq _f_ArelX __f_ArelX)) _f_ArelX)
  (:f_ArelY
   (&optional __f_ArelY)
   (if __f_ArelY (setq _f_ArelY __f_ArelY)) _f_ArelY)
  (:f_DistXStd
   (&optional __f_DistXStd)
   (if __f_DistXStd (setq _f_DistXStd __f_DistXStd)) _f_DistXStd)
  (:f_DistYStd
   (&optional __f_DistYStd)
   (if __f_DistYStd (setq _f_DistYStd __f_DistYStd)) _f_DistYStd)
  (:f_VrelXStd
   (&optional __f_VrelXStd)
   (if __f_VrelXStd (setq _f_VrelXStd __f_VrelXStd)) _f_VrelXStd)
  (:f_VrelYStd
   (&optional __f_VrelYStd)
   (if __f_VrelYStd (setq _f_VrelYStd __f_VrelYStd)) _f_VrelYStd)
  (:f_ArelXStd
   (&optional __f_ArelXStd)
   (if __f_ArelXStd (setq _f_ArelXStd __f_ArelXStd)) _f_ArelXStd)
  (:f_ArelYStd
   (&optional __f_ArelYStd)
   (if __f_ArelYStd (setq _f_ArelYStd __f_ArelYStd)) _f_ArelYStd)
  (:f_LDeltaX_left
   (&optional __f_LDeltaX_left)
   (if __f_LDeltaX_left (setq _f_LDeltaX_left __f_LDeltaX_left)) _f_LDeltaX_left)
  (:f_LDeltaX_mid
   (&optional __f_LDeltaX_mid)
   (if __f_LDeltaX_mid (setq _f_LDeltaX_mid __f_LDeltaX_mid)) _f_LDeltaX_mid)
  (:f_LDeltaX_right
   (&optional __f_LDeltaX_right)
   (if __f_LDeltaX_right (setq _f_LDeltaX_right __f_LDeltaX_right)) _f_LDeltaX_right)
  (:f_LDeltaY_left
   (&optional __f_LDeltaY_left)
   (if __f_LDeltaY_left (setq _f_LDeltaY_left __f_LDeltaY_left)) _f_LDeltaY_left)
  (:f_LDeltaY_mid
   (&optional __f_LDeltaY_mid)
   (if __f_LDeltaY_mid (setq _f_LDeltaY_mid __f_LDeltaY_mid)) _f_LDeltaY_mid)
  (:f_LDeltaY_right
   (&optional __f_LDeltaY_right)
   (if __f_LDeltaY_right (setq _f_LDeltaY_right __f_LDeltaY_right)) _f_LDeltaY_right)
  (:f_RCS
   (&optional __f_RCS)
   (if __f_RCS (setq _f_RCS __f_RCS)) _f_RCS)
  (:f_ObjectScore
   (&optional __f_ObjectScore)
   (if __f_ObjectScore (setq _f_ObjectScore __f_ObjectScore)) _f_ObjectScore)
  (:f_VabsX
   (&optional __f_VabsX)
   (if __f_VabsX (setq _f_VabsX __f_VabsX)) _f_VabsX)
  (:f_VabsY
   (&optional __f_VabsY)
   (if __f_VabsY (setq _f_VabsY __f_VabsY)) _f_VabsY)
  (:f_AabsX
   (&optional __f_AabsX)
   (if __f_AabsX (setq _f_AabsX __f_AabsX)) _f_AabsX)
  (:f_AabsY
   (&optional __f_AabsY)
   (if __f_AabsY (setq _f_AabsY __f_AabsY)) _f_AabsY)
  (:f_VabsXStd
   (&optional __f_VabsXStd)
   (if __f_VabsXStd (setq _f_VabsXStd __f_VabsXStd)) _f_VabsXStd)
  (:f_VabsYStd
   (&optional __f_VabsYStd)
   (if __f_VabsYStd (setq _f_VabsYStd __f_VabsYStd)) _f_VabsYStd)
  (:f_AabsXStd
   (&optional __f_AabsXStd)
   (if __f_AabsXStd (setq _f_AabsXStd __f_AabsXStd)) _f_AabsXStd)
  (:f_AabsYStd
   (&optional __f_AabsYStd)
   (if __f_AabsYStd (setq _f_AabsYStd __f_AabsYStd)) _f_AabsYStd)
  (:f_Orientation
   (&optional __f_Orientation)
   (if __f_Orientation (setq _f_Orientation __f_Orientation)) _f_Orientation)
  (:f_OrientationStd
   (&optional __f_OrientationStd)
   (if __f_OrientationStd (setq _f_OrientationStd __f_OrientationStd)) _f_OrientationStd)
  (:f_DistZ
   (&optional __f_DistZ)
   (if __f_DistZ (setq _f_DistZ __f_DistZ)) _f_DistZ)
  (:f_DistZStd
   (&optional __f_DistZStd)
   (if __f_DistZStd (setq _f_DistZStd __f_DistZStd)) _f_DistZStd)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; float32[] _f_DistX
    (* 4    (length _f_DistX)) 4
    ;; float32[] _f_DistY
    (* 4    (length _f_DistY)) 4
    ;; float32[] _f_VrelX
    (* 4    (length _f_VrelX)) 4
    ;; float32[] _f_VrelY
    (* 4    (length _f_VrelY)) 4
    ;; float32[] _f_ArelX
    (* 4    (length _f_ArelX)) 4
    ;; float32[] _f_ArelY
    (* 4    (length _f_ArelY)) 4
    ;; float32[] _f_DistXStd
    (* 4    (length _f_DistXStd)) 4
    ;; float32[] _f_DistYStd
    (* 4    (length _f_DistYStd)) 4
    ;; float32[] _f_VrelXStd
    (* 4    (length _f_VrelXStd)) 4
    ;; float32[] _f_VrelYStd
    (* 4    (length _f_VrelYStd)) 4
    ;; float32[] _f_ArelXStd
    (* 4    (length _f_ArelXStd)) 4
    ;; float32[] _f_ArelYStd
    (* 4    (length _f_ArelYStd)) 4
    ;; float32[] _f_LDeltaX_left
    (* 4    (length _f_LDeltaX_left)) 4
    ;; float32[] _f_LDeltaX_mid
    (* 4    (length _f_LDeltaX_mid)) 4
    ;; float32[] _f_LDeltaX_right
    (* 4    (length _f_LDeltaX_right)) 4
    ;; float32[] _f_LDeltaY_left
    (* 4    (length _f_LDeltaY_left)) 4
    ;; float32[] _f_LDeltaY_mid
    (* 4    (length _f_LDeltaY_mid)) 4
    ;; float32[] _f_LDeltaY_right
    (* 4    (length _f_LDeltaY_right)) 4
    ;; float32[] _f_RCS
    (* 4    (length _f_RCS)) 4
    ;; float32[] _f_ObjectScore
    (* 4    (length _f_ObjectScore)) 4
    ;; float32[] _f_VabsX
    (* 4    (length _f_VabsX)) 4
    ;; float32[] _f_VabsY
    (* 4    (length _f_VabsY)) 4
    ;; float32[] _f_AabsX
    (* 4    (length _f_AabsX)) 4
    ;; float32[] _f_AabsY
    (* 4    (length _f_AabsY)) 4
    ;; float32[] _f_VabsXStd
    (* 4    (length _f_VabsXStd)) 4
    ;; float32[] _f_VabsYStd
    (* 4    (length _f_VabsYStd)) 4
    ;; float32[] _f_AabsXStd
    (* 4    (length _f_AabsXStd)) 4
    ;; float32[] _f_AabsYStd
    (* 4    (length _f_AabsYStd)) 4
    ;; float32[] _f_Orientation
    (* 4    (length _f_Orientation)) 4
    ;; float32[] _f_OrientationStd
    (* 4    (length _f_OrientationStd)) 4
    ;; float32[] _f_DistZ
    (* 4    (length _f_DistZ)) 4
    ;; float32[] _f_DistZStd
    (* 4    (length _f_DistZStd)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; float32[] _f_DistX
     (write-long (length _f_DistX) s)
     (dotimes (i (length _f_DistX))
       (sys::poke (elt _f_DistX i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_DistY
     (write-long (length _f_DistY) s)
     (dotimes (i (length _f_DistY))
       (sys::poke (elt _f_DistY i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_VrelX
     (write-long (length _f_VrelX) s)
     (dotimes (i (length _f_VrelX))
       (sys::poke (elt _f_VrelX i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_VrelY
     (write-long (length _f_VrelY) s)
     (dotimes (i (length _f_VrelY))
       (sys::poke (elt _f_VrelY i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_ArelX
     (write-long (length _f_ArelX) s)
     (dotimes (i (length _f_ArelX))
       (sys::poke (elt _f_ArelX i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_ArelY
     (write-long (length _f_ArelY) s)
     (dotimes (i (length _f_ArelY))
       (sys::poke (elt _f_ArelY i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_DistXStd
     (write-long (length _f_DistXStd) s)
     (dotimes (i (length _f_DistXStd))
       (sys::poke (elt _f_DistXStd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_DistYStd
     (write-long (length _f_DistYStd) s)
     (dotimes (i (length _f_DistYStd))
       (sys::poke (elt _f_DistYStd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_VrelXStd
     (write-long (length _f_VrelXStd) s)
     (dotimes (i (length _f_VrelXStd))
       (sys::poke (elt _f_VrelXStd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_VrelYStd
     (write-long (length _f_VrelYStd) s)
     (dotimes (i (length _f_VrelYStd))
       (sys::poke (elt _f_VrelYStd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_ArelXStd
     (write-long (length _f_ArelXStd) s)
     (dotimes (i (length _f_ArelXStd))
       (sys::poke (elt _f_ArelXStd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_ArelYStd
     (write-long (length _f_ArelYStd) s)
     (dotimes (i (length _f_ArelYStd))
       (sys::poke (elt _f_ArelYStd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_LDeltaX_left
     (write-long (length _f_LDeltaX_left) s)
     (dotimes (i (length _f_LDeltaX_left))
       (sys::poke (elt _f_LDeltaX_left i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_LDeltaX_mid
     (write-long (length _f_LDeltaX_mid) s)
     (dotimes (i (length _f_LDeltaX_mid))
       (sys::poke (elt _f_LDeltaX_mid i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_LDeltaX_right
     (write-long (length _f_LDeltaX_right) s)
     (dotimes (i (length _f_LDeltaX_right))
       (sys::poke (elt _f_LDeltaX_right i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_LDeltaY_left
     (write-long (length _f_LDeltaY_left) s)
     (dotimes (i (length _f_LDeltaY_left))
       (sys::poke (elt _f_LDeltaY_left i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_LDeltaY_mid
     (write-long (length _f_LDeltaY_mid) s)
     (dotimes (i (length _f_LDeltaY_mid))
       (sys::poke (elt _f_LDeltaY_mid i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_LDeltaY_right
     (write-long (length _f_LDeltaY_right) s)
     (dotimes (i (length _f_LDeltaY_right))
       (sys::poke (elt _f_LDeltaY_right i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_RCS
     (write-long (length _f_RCS) s)
     (dotimes (i (length _f_RCS))
       (sys::poke (elt _f_RCS i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_ObjectScore
     (write-long (length _f_ObjectScore) s)
     (dotimes (i (length _f_ObjectScore))
       (sys::poke (elt _f_ObjectScore i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_VabsX
     (write-long (length _f_VabsX) s)
     (dotimes (i (length _f_VabsX))
       (sys::poke (elt _f_VabsX i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_VabsY
     (write-long (length _f_VabsY) s)
     (dotimes (i (length _f_VabsY))
       (sys::poke (elt _f_VabsY i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_AabsX
     (write-long (length _f_AabsX) s)
     (dotimes (i (length _f_AabsX))
       (sys::poke (elt _f_AabsX i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_AabsY
     (write-long (length _f_AabsY) s)
     (dotimes (i (length _f_AabsY))
       (sys::poke (elt _f_AabsY i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_VabsXStd
     (write-long (length _f_VabsXStd) s)
     (dotimes (i (length _f_VabsXStd))
       (sys::poke (elt _f_VabsXStd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_VabsYStd
     (write-long (length _f_VabsYStd) s)
     (dotimes (i (length _f_VabsYStd))
       (sys::poke (elt _f_VabsYStd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_AabsXStd
     (write-long (length _f_AabsXStd) s)
     (dotimes (i (length _f_AabsXStd))
       (sys::poke (elt _f_AabsXStd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_AabsYStd
     (write-long (length _f_AabsYStd) s)
     (dotimes (i (length _f_AabsYStd))
       (sys::poke (elt _f_AabsYStd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_Orientation
     (write-long (length _f_Orientation) s)
     (dotimes (i (length _f_Orientation))
       (sys::poke (elt _f_Orientation i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_OrientationStd
     (write-long (length _f_OrientationStd) s)
     (dotimes (i (length _f_OrientationStd))
       (sys::poke (elt _f_OrientationStd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_DistZ
     (write-long (length _f_DistZ) s)
     (dotimes (i (length _f_DistZ))
       (sys::poke (elt _f_DistZ i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _f_DistZStd
     (write-long (length _f_DistZStd) s)
     (dotimes (i (length _f_DistZStd))
       (sys::poke (elt _f_DistZStd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _f_DistX
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_DistX (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_DistX i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_DistY
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_DistY (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_DistY i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_VrelX
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_VrelX (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_VrelX i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_VrelY
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_VrelY (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_VrelY i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_ArelX
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_ArelX (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_ArelX i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_ArelY
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_ArelY (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_ArelY i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_DistXStd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_DistXStd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_DistXStd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_DistYStd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_DistYStd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_DistYStd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_VrelXStd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_VrelXStd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_VrelXStd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_VrelYStd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_VrelYStd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_VrelYStd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_ArelXStd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_ArelXStd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_ArelXStd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_ArelYStd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_ArelYStd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_ArelYStd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_LDeltaX_left
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_LDeltaX_left (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_LDeltaX_left i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_LDeltaX_mid
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_LDeltaX_mid (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_LDeltaX_mid i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_LDeltaX_right
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_LDeltaX_right (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_LDeltaX_right i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_LDeltaY_left
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_LDeltaY_left (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_LDeltaY_left i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_LDeltaY_mid
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_LDeltaY_mid (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_LDeltaY_mid i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_LDeltaY_right
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_LDeltaY_right (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_LDeltaY_right i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_RCS
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_RCS (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_RCS i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_ObjectScore
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_ObjectScore (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_ObjectScore i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_VabsX
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_VabsX (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_VabsX i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_VabsY
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_VabsY (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_VabsY i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_AabsX
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_AabsX (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_AabsX i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_AabsY
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_AabsY (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_AabsY i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_VabsXStd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_VabsXStd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_VabsXStd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_VabsYStd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_VabsYStd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_VabsYStd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_AabsXStd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_AabsXStd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_AabsXStd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_AabsYStd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_AabsYStd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_AabsYStd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_Orientation
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_Orientation (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_Orientation i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_OrientationStd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_OrientationStd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_OrientationStd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_DistZ
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_DistZ (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_DistZ i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _f_DistZStd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_DistZStd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_DistZStd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get conti_radar::radar_obj :md5sum-) "d139e1dca278064cbcdfddd5e7dfb3bb")
(setf (get conti_radar::radar_obj :datatype-) "conti_radar/radar_obj")
(setf (get conti_radar::radar_obj :definition-)
      "int32 id
float32[] f_DistX
float32[] f_DistY
float32[] f_VrelX
float32[] f_VrelY
float32[] f_ArelX
float32[] f_ArelY
float32[] f_DistXStd
float32[] f_DistYStd
float32[] f_VrelXStd
float32[] f_VrelYStd
float32[] f_ArelXStd
float32[] f_ArelYStd
float32[] f_LDeltaX_left
float32[] f_LDeltaX_mid
float32[] f_LDeltaX_right
float32[] f_LDeltaY_left

float32[] f_LDeltaY_mid
float32[] f_LDeltaY_right
float32[] f_RCS
float32[] f_ObjectScore
float32[] f_VabsX
float32[] f_VabsY
float32[] f_AabsX
float32[] f_AabsY
float32[] f_VabsXStd
float32[] f_VabsYStd
float32[] f_AabsXStd
float32[] f_AabsYStd

float32[] f_Orientation
float32[] f_OrientationStd
float32[] f_DistZ
float32[] f_DistZStd
")



(provide :conti_radar/radar_obj "d139e1dca278064cbcdfddd5e7dfb3bb")


