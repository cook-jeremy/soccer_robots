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
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('control_gui', anonymous=True)
        self.rate = rospy.Rate(10)  # 10hz

    def gui_init(self):
        self.root = Tkinter.Tk()
        Tkinter.Button(self.root, text="Hello", command=self.helloCallBack).grid(row=0, column=0)
        Tkinter.Button(self.root, text="Hello", command=self.helloCallBack).grid(row=1, column=0)
        Tkinter.Button(self.root, text="Hello", command=self.helloCallBack).grid(row=2, column=0)
        Tkinter.Button(self.root, text="Hello", command=self.helloCallBack).grid(row=3, column=0)

        Tkinter.Button(self.root, text="Hello", command=self.helloCallBack).grid(row=0, column=1)
        Tkinter.Button(self.root, text="Hello", command=self.helloCallBack).grid(row=1, column=1)
        Tkinter.Button(self.root, text="Hello", command=self.helloCallBack).grid(row=2, column=1)
        Tkinter.Button(self.root, text="Hello", command=self.helloCallBack).grid(row=3, column=1)

    def helloCallBack(self):
       #tkMessageBox.showinfo( "Hello Python", "Hello World")
       hello_str = "hello world %s" % rospy.get_time()
       rospy.loginfo(hello_str)
       self.pub.publish(hello_str)

    def main(self):
        while not rospy.is_shutdown():
            self.root.mainloop()

if __name__ == '__main__':
    try:
        c = Control()
        c.main()
    except rospy.ROSInterruptException:
        pass