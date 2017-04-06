#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
from perception_core.perception_base import PerceptionBase

from std_msgs.msg import Float64
from google_cloud_speech.msg import RecognizedWord


class SpeechRecognizer(PerceptionBase):
    def __init__(self):
        super(SpeechRecognizer, self).__init__("sp_speech_recognizer")

        rospy.Subscriber('recognized_word', RecognizedWord, self.handle_recognized_word)
        rospy.loginfo('[%s] initialze done...'%rospy.get_name())
        rospy.spin()

    def handle_recognized_word(self, msg):
        write_data = self.conf_data['speech_recognition']['data']

        write_data['recognized_word'] = msg.recognized_word
        write_data['confidence'] = msg.confidence

        self.save_to_memory(self.conf_data.keys()[0], data=write_data)
        self.raise_event(self.conf_data.keys()[0], 'speech_recognized')


if __name__ == '__main__':
    m = SpeechRecognizer()
