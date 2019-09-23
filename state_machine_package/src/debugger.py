#!/usr/bin/env python

import rospy
from state_machine_package.msg import commandRequest, commandResponse
from utils import color
import time

class message_updater():
    def __init__(self):
        self.request = commandRequest()
        self.request.request_number = 1
        self.request.request_type = 'change_state'
        self.response = commandResponse()
        self.sub = 0
        self.pub = rospy.Publisher('/command_response', commandResponse, queue_size = 1)
        self.rate = rospy.Rate(10)
        self.ok = True

    def received(self, msg):
        if msg.request_number != self.request.request_number: #se ho un dato nuovo
            self.request = msg
            rospy.loginfo(color.BOLD + color.YELLOW + 'RECEIVED REQUEST NUMBER: ' + str(self.request.request_number) + color.END)
            self.ok = True

    def asknew(self):
        '''RESPONSE:
           header
           int64 request_number
           string request_type
           int16 command

           REQUEST
           header
           int64 request_number
           string request_type
           active_state'''

        new = raw_input(color.BOLD + color.CYAN + 'NEW COMMAND: ' + color.END)
        self.response.request_number = self.request.request_number
        self.response.request_type = self.request.request_type
        self.response.command = int(new)
        self.response.header.stamp = rospy.Time.now()
        self.pub.publish(self.response)

    def execute(self):
        self.sub = rospy.Subscriber('/command_request', commandRequest, self.received, queue_size=1)
        time.sleep(1)
        if self.ok == True:
            self.asknew()
            rospy.loginfo(color.BOLD + color.YELLOW + '-- COMMAND SENT --' + color.END)
            self.ok = False
        self.rate.sleep()


def myhook():
    print(color.BOLD + color.RED + '\n -- keyboard interrupt, shutting down --' + color.END)

def main():
    rospy.init_node('debugger_node')
    updater = message_updater()
    while not rospy.is_shutdown():
        updater.execute()

    rospy.on_shutdown(myhook)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
