#!/usr/bin/env python
import roslib
roslib.load_manifest('localizer')
import rospy
import sys, select, termios, tty
from std_msgs.msg import String, Bool, ColorRGBA
import sensor_msgs.msg
import message_filters
import collections
from detection_msgs.msg import Detection
from localizer.srv import Localize
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point, Vector3
import math

counter = 0
maxDistance = 0.5
alreadyDetected = [] # an array containing n points, the first n detected that were considered the same detection
n = 3 # number of points required in order to determine that the detection is a face
detected = [] #an array with the already passed faces
class DetectionMapper():

    def camera_callback(self, camera_info):
        self.camera_infos.append(camera_info)

    def distance(self, p1, p2):
	xd = p1.x - p2.x
	yd = p1.y - p2.y
	dd = xd ** 2 + yd ** 2
	res = math.sqrt(dd)
	return res 


    def __init__(self): 
   	counter = 0
	maxDistance = 0.5
	alreadyDetected = [] # an array containing n points, the first n detected that were considered the same detection
	n = 3 # number of points required in order to determine that the detection is a face
	detected = []
	self.markers = MarkerArray()
        self.marker_id_counter = 0
        self.region_scope = rospy.get_param('~region', 3)
        self.buffer_size = rospy.get_param('~camera_buffer_size', 50)
        rospy.wait_for_service('localizer/localize')

        self.camera_infos = collections.deque(maxlen = self.buffer_size)
        self.detections_sub = message_filters.Subscriber('detections', Detection)
	self.detections_sub.registerCallback(self.detections_callback)

        self.camera_sub = message_filters.Subscriber('camera_info', CameraInfo)
	self.camera_sub.registerCallback(self.camera_callback)

        self.localize = rospy.ServiceProxy('localizer/localize', Localize)
        self.markers_pub = rospy.Publisher('markers', MarkerArray)

    def detections_callback(self, detection):
	global counter
	global maxDistance #the maximal distance between two points required for them to be considered the same detection	
	global alreadyDetected 
	global n 
	global lastAdded
	global detected
	

        u = detection.x + detection.width / 2
        v = detection.y + detection.height / 2

        camera_info = None
        best_time = 100
        for ci in self.camera_infos:
            time = abs(ci.header.stamp.to_sec() - detection.header.stamp.to_sec())
            if time < best_time:
                camera_info = ci
                best_time = time

        if not camera_info or best_time > 1:
            return

        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)

        point = Point(((u - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(),
             ((v - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), 1)

        localization = self.localize(detection.header, point, self.region_scope)

        if not localization:
            return

	print("Found a face.")
	if counter == 0:
		print("Counter is zero.")
		counter+=1
		lastAdded = point
		print("Adding the point to the array.")
		alreadyDetected.append(point)
		print(len(alreadyDetected))
		print("Added the point to the array.")
		
	else:
		print("counter isn't zero.")
		dist = self.distance(point, lastAdded)
		print("Distance is ", dist)
		if dist <= maxDistance:
		     if counter < 2:
				print("Adding the point to the array.")
				alreadyDetected.append(point)
				print(len(alreadyDetected))			
				# alreadyDetected[counter] = point
				lastAdded = point
				counter += 1
		     elif counter == 2:
				counter += 1
				 
				#check here if the face has been recognised before -> that's in detected[]
				
				beenDetected = False
				
				for p in detected:
					if self.distance(p, point) <= maxDistance:
						beenDetected = True
						break
						
				if(beenDetected == False):
					
					print("Adding one single marker.")
					marker = Marker()
					marker.header.stamp = detection.header.stamp
					marker.header.frame_id = detection.header.frame_id
					marker.pose = localization.pose
					marker.type = Marker.CUBE
					marker.action = Marker.ADD
					marker.frame_locked = False
					marker.lifetime = rospy.Duration.from_sec(1)
					marker.id = self.marker_id_counter
					marker.scale = Vector3(0.1, 0.1, 0.1)
					marker.color = ColorRGBA(1, 0, 0, 1)
					self.markers.markers.append(marker)
					self.marker_id_counter += 1
					print("Added marker.")
		else:	
			detected.append(lastAdded)
			counter = 0	
			alreadyDetected = []
			alreadyDetected.append(point)
			lastAdded = point

       

    def flush(self):
        if not self.markers.markers:
            self.markers = MarkerArray()
            return
        self.markers_pub.publish(self.markers)
        #self.markers = MarkerArray()

   
if __name__ == '__main__':

        rospy.init_node('mapper')

        try:
            mapper = DetectionMapper()
            r = rospy.Rate(30)
            while not rospy.is_shutdown():
                mapper.flush()
                r.sleep()
        except rospy.ROSInterruptException: pass

