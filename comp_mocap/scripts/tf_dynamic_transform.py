#!/usr/bin/env python
# Author: Daniel Bessler

from threading import Thread, Lock
from math import pi

import tf2_ros
import tf
import rospy
from geometry_msgs.msg import TransformStamped

import Tkinter as tk

class DynamicPublisher:
    """ Simple TF broadcaster that allows to dynamically change the transformation
    between 2 TF frames. """
    
    def __init__(self):
        self.mutex = Lock()
        self.pub = tf2_ros.TransformBroadcaster()
        self.pose_msg = TransformStamped()
        self.setFrame("map")
        self.setChildFrame("shoulder_kinect_link")
        self.setTranslation(0.0,0.0,0.0)
        self.setQuaternion(0.0,0.0,0.0,0.0)
        self.interval = 100
    
    def setFrame(self,name):
        """ The root frame for the transformation. """
        self.mutex.acquire()
        try:
            self.pose_msg.header.frame_id = name
        finally:
            self.mutex.release()
    
    def setChildFrame(self,name):
        """ The child frame for the transformation. """
        self.mutex.acquire()
        try:
            self.pose_msg.child_frame_id = name
        finally:
            self.mutex.release()
    
    def setTranslation(self,x,y,z):
        """ The translation between root and child frame. """
        self.mutex.acquire()
        try:
            self.pose_msg.transform.translation.x = x
            self.pose_msg.transform.translation.y = y
            self.pose_msg.transform.translation.z = z
        finally:
            self.mutex.release()
    
    def setQuaternion(self,x,y,z,w):
        """ The rotation between root and child frame. """
        self.mutex.acquire()
        try:
            self.pose_msg.transform.rotation.x = x
            self.pose_msg.transform.rotation.y = y
            self.pose_msg.transform.rotation.z = z
            self.pose_msg.transform.rotation.w = w
        finally:
            self.mutex.release()

    def publish(self):
        """ Publish the TF message that defines a transform between
        parent and child TF frame. """
        self.mutex.acquire()
        try:
            self.pose_msg.header.stamp = rospy.Time.now() + rospy.Duration(0.5)
            self.pub.sendTransform(self.pose_msg)
        finally:
            self.mutex.release()

    def run(self):
        """ Run infinite. """
        r = rospy.Rate(self.interval)
        while not rospy.is_shutdown():
            self.publish()
            r.sleep()

class DynamicPublisherGui(tk.Frame):
    WIDGET_LENGTH      = 400 # pixel
    SLIDER_RESOLUTION  = 0.01
    TRANSLATION_BOUNDS = (-2.0,2.0)
    ROTATION_BOUNDS    = (0.0,2.0*pi)

    def __init__(self, master=None, title="Dynamic TF Publisher"):
        tk.Frame.__init__(self, master)
        self.master.resizable(width=False,height=False)
        self.master.title(title)
        
        rospy.init_node('dynamic_tf_publisher')
        self.publisher = DynamicPublisher()
        self.publisherThread = Thread(target=self.publisher.run)
        
        self.grid()
        self.translationSlider = {}
        self.rotationSlider = {}
        self.createWidgets()

    def updateFrame(self, *args):
        self.publisher.setFrame(self.frameVar.get())

    def updateChildFrame(self, *args):
        self.publisher.setChildFrame(self.childFrameVar.get())

    def updateInterval(self, *args):
        self.publisher.interval = self.intervalSlider.get()

    def updateTranslation(self, *args):
        self.publisher.setTranslation(
            self.translationSlider["x"].get(),
            self.translationSlider["y"].get(),
            self.translationSlider["z"].get())

    def updateRotation(self, *args):
        pitch = self.rotationSlider["pitch"].get()
        roll  = self.rotationSlider["roll"].get()
        yaw   = self.rotationSlider["yaw"].get()
        rot = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        self.publisher.setQuaternion(rot[0],rot[1],rot[2],rot[3])

    def createTranslationSlider(self, comp, r):
        slider = tk.Scale(self,
            label="translation."+comp,
            from_=self.TRANSLATION_BOUNDS[0],
            to=self.TRANSLATION_BOUNDS[1],
            orient=tk.HORIZONTAL,
            resolution=self.SLIDER_RESOLUTION,
            length=self.WIDGET_LENGTH,
            command=self.updateTranslation)
        slider.grid(column=1,row=r,padx=2)
        self.translationSlider[comp] = slider

    def createRotationSlider(self, comp, r):
        slider = tk.Scale(self,
            label="rotation."+comp,
            from_=self.ROTATION_BOUNDS[0],
            to=self.ROTATION_BOUNDS[1],
            orient=tk.HORIZONTAL,
            resolution=self.SLIDER_RESOLUTION,
            length=self.WIDGET_LENGTH,
            command=self.updateRotation)
        slider.grid(column=0,row=r,padx=2)
        self.rotationSlider[comp] = slider

    def createFrameEntry(self, c, txt, val, handler):
        tk.Label(self, text=txt).grid(row=0, column=c, sticky=tk.W, padx=8)
        var = tk.StringVar()
        var.set(val)
        e = tk.Entry(self,textvariable=var)
        e.grid(row=1, column=c, padx=2, sticky=tk.W)
        var.trace("w", handler)
        return var
    
    def createSeparator(self):
        sep = tk.Frame(self, height=2,
            width=2*self.WIDGET_LENGTH+4,
            bd=1, bg="grey", relief=tk.SUNKEN)
        sep.grid(columnspan=2, padx=0, pady=5)

    def createWidgets(self):
        self.frameVar = self.createFrameEntry(0, "frame",
            self.publisher.pose_msg.header.frame_id, self.updateFrame)
        self.childFrameVar = self.createFrameEntry(1, "child-frame",
            self.publisher.pose_msg.child_frame_id, self.updateChildFrame)
        
        self.intervalSlider = tk.Scale(self,
            label="interval",
            from_=1, to=1000,
            orient=tk.HORIZONTAL,
            length=self.WIDGET_LENGTH,
            command=self.updateInterval)
        self.intervalSlider.set(self.publisher.interval)
        self.intervalSlider.grid(column=0,row=3,padx=2)
        self.createSeparator()
        
        for (n,m,r) in [("x","pitch",5),("y","roll",6),("z","yaw",7)]:
            self.createTranslationSlider(n,r)
            self.createRotationSlider(m,r)
        
        self.quitButton = tk.Button(self, text='Quit', command=self.quit)
        self.quitButton.grid(column=1, pady=4, padx=2, sticky=tk.E)

    def start(self):
        self.publisherThread.start()
        tk.Frame.mainloop(self)
        rospy.signal_shutdown(0)

if __name__ == '__main__':
    gui = DynamicPublisherGui()
    gui.start()

