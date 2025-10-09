;; Auto-generated. Do not edit!


(when (boundp 'camera::message)
  (if (not (find-package "CAMERA"))
    (make-package "CAMERA"))
  (shadow 'message (find-package "CAMERA")))
(unless (find-package "CAMERA::MESSAGE")
  (make-package "CAMERA::MESSAGE"))
(unless (find-package "CAMERA::MESSAGEREQUEST")
  (make-package "CAMERA::MESSAGEREQUEST"))
(unless (find-package "CAMERA::MESSAGERESPONSE")
  (make-package "CAMERA::MESSAGERESPONSE"))

(in-package "ROS")





(defclass camera::messageRequest
  :super ros::object
  :slots (_if_request ))

(defmethod camera::messageRequest
  (:init
   (&key
    ((:if_request __if_request) 0)
    )
   (send-super :init)
   (setq _if_request (round __if_request))
   self)
  (:if_request
   (&optional __if_request)
   (if __if_request (setq _if_request __if_request)) _if_request)
  (:serialization-length
   ()
   (+
    ;; int32 _if_request
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _if_request
       (write-long _if_request s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _if_request
     (setq _if_request (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass camera::messageResponse
  :super ros::object
  :slots (_x_p _y_q _depth ))

(defmethod camera::messageResponse
  (:init
   (&key
    ((:x_p __x_p) 0.0)
    ((:y_q __y_q) 0.0)
    ((:depth __depth) 0.0)
    )
   (send-super :init)
   (setq _x_p (float __x_p))
   (setq _y_q (float __y_q))
   (setq _depth (float __depth))
   self)
  (:x_p
   (&optional __x_p)
   (if __x_p (setq _x_p __x_p)) _x_p)
  (:y_q
   (&optional __y_q)
   (if __y_q (setq _y_q __y_q)) _y_q)
  (:depth
   (&optional __depth)
   (if __depth (setq _depth __depth)) _depth)
  (:serialization-length
   ()
   (+
    ;; float64 _x_p
    8
    ;; float64 _y_q
    8
    ;; float64 _depth
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x_p
       (sys::poke _x_p (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_q
       (sys::poke _y_q (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _depth
       (sys::poke _depth (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x_p
     (setq _x_p (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_q
     (setq _y_q (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _depth
     (setq _depth (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass camera::message
  :super ros::object
  :slots ())

(setf (get camera::message :md5sum-) "b9d1c9fee0054923e6d8acbb284d7b0b")
(setf (get camera::message :datatype-) "camera/message")
(setf (get camera::message :request) camera::messageRequest)
(setf (get camera::message :response) camera::messageResponse)

(defmethod camera::messageRequest
  (:response () (instance camera::messageResponse :init)))

(setf (get camera::messageRequest :md5sum-) "b9d1c9fee0054923e6d8acbb284d7b0b")
(setf (get camera::messageRequest :datatype-) "camera/messageRequest")
(setf (get camera::messageRequest :definition-)
      "# 客户端请求时发送的两个数字
int32 if_request
---
# 服务器响应发送的数据
float64 x_p
float64 y_q
float64 depth

")

(setf (get camera::messageResponse :md5sum-) "b9d1c9fee0054923e6d8acbb284d7b0b")
(setf (get camera::messageResponse :datatype-) "camera/messageResponse")
(setf (get camera::messageResponse :definition-)
      "# 客户端请求时发送的两个数字
int32 if_request
---
# 服务器响应发送的数据
float64 x_p
float64 y_q
float64 depth

")



(provide :camera/message "b9d1c9fee0054923e6d8acbb284d7b0b")


