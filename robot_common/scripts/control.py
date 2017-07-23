#!/usr/bin/env python
import Tkinter
import tkMessageBox
import rospy
from std_msgs.msg import String

class Control():
    
    def __init__(self):
        self.gui_init()
        self.ros_init()

    def ros_init(self):
        self.pub = rospy.Publisher('gui_commands', String, queue_size=10)
        rospy.init_node('control_gui', anonymous=True)
        self.rate = rospy.Rate(10)  # 10hz

    def gui_init(self):
        self.root = Tkinter.Tk()

        run = Tkinter.Button(self.root, text="Run", command=self.runCallBack, height=5, width=10)
        run.grid(row=0, column=0, padx=10, pady=10)

        stop = Tkinter.Button(self.root, text="Stop", command=self.stopCallBack, height=5, width=10)
        stop.grid(row=0, column=1, padx=10, pady=10)

        turn = Tkinter.Button(self.root, text="Turn North", command=self.turnNorthCallBack, height=5, width=10)
        turn.grid(row=0, column=2, padx=10, pady=10)

        turn = Tkinter.Button(self.root, text="Turn South", command=self.turnSouthCallBack, height=5, width=10)
        turn.grid(row=0, column=3, padx=10, pady=10)

    def stopCallBack(self):
        cmd = "stop"
        rospy.loginfo(cmd)
        self.pub.publish(cmd)

    def runCallBack(self):
        cmd = "run"
        rospy.loginfo(cmd)
        self.pub.publish(cmd)

    def turnNorthCallBack(self):
        cmd = "turn_0"
        rospy.loginfo(cmd)
        self.pub.publish(cmd)

    def turnSouthCallBack(self):
        cmd = "turn_180"
        rospy.loginfo(cmd)
        self.pub.publish(cmd)

    def main(self):
        while not rospy.is_shutdown():
            self.root.mainloop()

if __name__ == '__main__':
    try:
        c = Control()
        c.main()
    except rospy.ROSInterruptException:
        pass