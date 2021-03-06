#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
from perception_base.perception_base import PerceptionBase

from std_msgs.msg import Float64, Empty, Bool
from mhri_social_msgs.msg import RecognizedWord


class SpeechRecognizer(PerceptionBase):
    def __init__(self):
        super(SpeechRecognizer, self).__init__("speech_recognizer")

        rospy.Subscriber('recognized_word', RecognizedWord, self.handle_recognized_word)
        rospy.loginfo('[%s] initialze done...'%rospy.get_name())

        self.pub_enable_google_speech = rospy.Publisher('enable_recognition', Bool, queue_size=5)
        rospy.spin()

    def handle_recognized_word(self, msg):
        write_data = self.conf_data['speech_recognition']['data']

        write_data['recognized_word'] = msg.recognized_word
        write_data['confidence'] = msg.confidence

        self.save_to_memory(self.conf_data.keys()[0], data=write_data)
        self.raise_event(self.conf_data.keys()[0], 'speech_recognized')

    def handle_start_perception(self, msg):
        self.pub_enable_google_speech.publish(True)
        super(SpeechRecognizer, self).handle_start_perception(msg)


    def handle_stop_perception(self, msg):
        self.pub_enable_google_speech.publish(False)
        super(SpeechRecognizer, self).handle_stop_perception(msg)


if __name__ == '__main__':
    m = SpeechRecognizer()
