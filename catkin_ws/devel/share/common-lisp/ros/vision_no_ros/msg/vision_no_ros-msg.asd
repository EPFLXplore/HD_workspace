
(cl:in-package :asdf)

(defsystem "vision_no_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "vector_msg" :depends-on ("_package_vector_msg"))
    (:file "_package_vector_msg" :depends-on ("_package"))
  ))