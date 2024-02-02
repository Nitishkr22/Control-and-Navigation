
(cl:in-package :asdf)

(defsystem "conti_radar-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "radar_msgs" :depends-on ("_package_radar_msgs"))
    (:file "_package_radar_msgs" :depends-on ("_package"))
  ))