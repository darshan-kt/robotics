;; Auto-generated. Do not edit!


(when (boundp 'tester::WordCount)
  (if (not (find-package "TESTER"))
    (make-package "TESTER"))
  (shadow 'WordCount (find-package "TESTER")))
(unless (find-package "TESTER::WORDCOUNT")
  (make-package "TESTER::WORDCOUNT"))
(unless (find-package "TESTER::WORDCOUNTREQUEST")
  (make-package "TESTER::WORDCOUNTREQUEST"))
(unless (find-package "TESTER::WORDCOUNTRESPONSE")
  (make-package "TESTER::WORDCOUNTRESPONSE"))

(in-package "ROS")





(defclass tester::WordCountRequest
  :super ros::object
  :slots (_words ))

(defmethod tester::WordCountRequest
  (:init
   (&key
    ((:words __words) "")
    )
   (send-super :init)
   (setq _words (string __words))
   self)
  (:words
   (&optional __words)
   (if __words (setq _words __words)) _words)
  (:serialization-length
   ()
   (+
    ;; string _words
    4 (length _words)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _words
       (write-long (length _words) s) (princ _words s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _words
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _words (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass tester::WordCountResponse
  :super ros::object
  :slots (_count ))

(defmethod tester::WordCountResponse
  (:init
   (&key
    ((:count __count) 0)
    )
   (send-super :init)
   (setq _count (round __count))
   self)
  (:count
   (&optional __count)
   (if __count (setq _count __count)) _count)
  (:serialization-length
   ()
   (+
    ;; uint32 _count
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _count
       (write-long _count s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _count
     (setq _count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass tester::WordCount
  :super ros::object
  :slots ())

(setf (get tester::WordCount :md5sum-) "58903d21a3264f3408d79ba79e9f7c7e")
(setf (get tester::WordCount :datatype-) "tester/WordCount")
(setf (get tester::WordCount :request) tester::WordCountRequest)
(setf (get tester::WordCount :response) tester::WordCountResponse)

(defmethod tester::WordCountRequest
  (:response () (instance tester::WordCountResponse :init)))

(setf (get tester::WordCountRequest :md5sum-) "58903d21a3264f3408d79ba79e9f7c7e")
(setf (get tester::WordCountRequest :datatype-) "tester/WordCountRequest")
(setf (get tester::WordCountRequest :definition-)
      "string words
---
uint32 count
")

(setf (get tester::WordCountResponse :md5sum-) "58903d21a3264f3408d79ba79e9f7c7e")
(setf (get tester::WordCountResponse :datatype-) "tester/WordCountResponse")
(setf (get tester::WordCountResponse :definition-)
      "string words
---
uint32 count
")



(provide :tester/WordCount "58903d21a3264f3408d79ba79e9f7c7e")


