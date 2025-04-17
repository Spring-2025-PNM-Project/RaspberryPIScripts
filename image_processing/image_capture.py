from picamera2 import Picamera2

picam2 = Picamera2()
picam2.start_and_capture_file("marvin.jpg")
picam2.close()