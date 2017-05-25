#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
from perception_core.perception_base import PerceptionBase

from mhri_social_msgs.msg import FaceDetection3D

class FaceDetector(PerceptionBase):
    def __init__(self):
        super(FaceDetector, self).__init__("face_detector")

        self.last_num_of_detected = 0
        self.confidence_count = 0
        self.avg_pose = {}
        self.loop_initialized = False
        self.loop_count = 0
        self.notified = False
        self.idle_count = 0;

        rospy.Subscriber('face_detected', FaceDetection3D, self.handle_face_detection)
        rospy.loginfo('[%s] initialze done...'%rospy.get_name())
        rospy.spin()

    def handle_face_detection(self, msg):
        if self.last_num_of_detected != msg.num_of_detected:
            self.confidence_count += 1
            if self.confidence_count > 20:
                self.last_num_of_detected = msg.num_of_detected
                self.confidence_count = 0
                self.notified = False

        if self.last_num_of_detected > 0:
            if not self.loop_initialized and msg.num_of_detected > 0:
                for person in range(self.last_num_of_detected):
                    self.avg_pose['Person%d'%person] = [msg.faces_pose[person].point.x, msg.faces_pose[person].point.y, msg.faces_pose[person].point.z]
                self.loop_initialized = True
            else:
                if msg.num_of_detected != self.last_num_of_detected:
                    self.loop_count = 0
                    self.loop_initialized = False
                    self.avg_pose = {}
                    return

                for person in range(self.last_num_of_detected):
                    self.avg_pose['Person%d'%person][0] += msg.faces_pose[person].point.x
                    self.avg_pose['Person%d'%person][1] += msg.faces_pose[person].point.y
                    self.avg_pose['Person%d'%person][2] += msg.faces_pose[person].point.z

                self.loop_count += 1
                if self.loop_count > 10:
                    for person in range(self.last_num_of_detected):
                        pose_x = self.avg_pose['Person%d'%person][0] / 10.0
                        pose_y = self.avg_pose['Person%d'%person][1] / 10.0
                        pose_z = self.avg_pose['Person%d'%person][2] / 10.0

                        write_data = self.conf_data['face_detection']['data']
                        write_data['num_of_detected'] = msg.num_of_detected
                        self.save_to_memory('face_detection', data=write_data)

                        if not self.notified:
                            self.raise_event('face_detection', 'face_detected')
                            self.notified = True

                        write_data = self.conf_data['persons']['data']
                        write_data['name'] = 'Person%d'%person
                        write_data['description'] = 'Person%d'%person
                        write_data['xyz'] = [pose_x, pose_y, pose_z]
                        write_data['frame_id'] = msg.faces_pose[person].header.frame_id

                        self.save_to_memory('persons', data=write_data)

                        self.loop_count = 0
                        self.loop_initialized = False
                        self.avg_pose['Person%d'%person] = [0.0, 0.0, 0.0]

        else:
            self.idle_count += 1
            if self.idle_count > 20:
                write_data = self.conf_data['face_detection']['data']
                write_data['num_of_detected'] = 0
                self.save_to_memory('face_detection', data=write_data)

if __name__ == '__main__':
    m = FaceDetector()
