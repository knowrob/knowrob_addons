#!/usr/bin/env python
# Author: Benjamin Brieber

import socket
import threading
import numpy
from struct import *

import signal
import sys


##get the ID of he parent
def get_segment_parent_id(id):
  ##attached to global frame
  if(id==1):
    return 0
  ##attached to shoulders
  elif(id == 8 or id == 12):
    return 5
  ##attached to pelvis
  elif(id == 16 or id ==20):
    return 1
  ##attached to the bone in the list before that bone (default)
  elif(id in range(2,25)):
    return id-1
  ##NOT DEFINED (MAYBE THROW EXCEPTION) 
  return -1


##get the name of he parent () default is the name of the global tf_frame
def get_segment_parent_name(id,default):
  ##attached to global frame
  if(id==1):
    return default
  ##attached to shoulders
  elif(id == 8 or id == 12):
    return get_segment_name(5)
  ##attached to pelvis
  elif(id == 16 or id ==20):
    return get_segment_name(2)
  ##attached to the bone in the list before that bone (default)
  elif(id in range(2,25)):
    return get_segment_name(id-1)
  ##NOT DEFINED (MAYBE THROW EXCEPTION)
  return None


#Wraps index to ID (no Idea where this should be usefull)
def get_segment_parent_name_by_index(id, default):
  return get_segment_parent_name(id+1,default)
  

#returns the name of a segment
def get_segment_name(id):
  
    if id == 1:
      return "Pelvis"
    elif id == 2:
      return "L5"
    elif id == 3:
      return "L3"
    elif id == 4:
      return "T12"
    elif id == 5:
      return "T8"
    elif id == 6:
      return "Neck"
    elif id == 7:
      return "Head"
    elif id == 8:
      return "RightShoulder"
    elif id == 9:
      return "RightUpperArm"
    elif id == 10:
      return "RightForearm"
    elif id == 11:
      return "RightHand"
    elif id == 12:
      return "LeftShoulder"
    elif id == 13:
      return "LeftUpperArm"
    elif id == 14:
      return "LeftForearm"
    elif id == 15:
      return "LeftHand"
    elif id == 16:
      return "RightUpperLeg"
    elif id == 17:
      return "RightLowerLeg"
    elif id == 18:
      return "RightFoot"
    elif id == 19:
      return "RightToe"
    elif id == 20:
      return "LeftUpperLeg"
    elif id == 21:
      return "LeftLowerLeg"
    elif id == 21:
      return "LeftFoot"
      ##no 22 But seems to be neccesary??? WHY HERE
    elif id == 22:
      return "LeftFoot"
    elif id == 23:
      return "LeftToe"
    elif id == 25:
      return "Prop1"
    elif id == 26:
      return "Prop2"
    elif id == 27:
      return "Prop3"
    elif id == 28:
      return "Prop4"
    return None
    
##wrapper for index (not used)
def get_segment_name_by_index(index):
  return get_segment_name(index+1)


##Dummy Class for Observer Pattern...
##Observing classes habe to implement an update Function
class Subject(object):
  def __init__(self):
    self._observers = []

  ##attach a new Observer
  def attach(self, observer):
    if not observer in self._observers:
      self._observers.append(observer)

  ##detach an Observer
  def detach(self, observer):
    try:
      self._observers.remove(observer)
    except ValueError:
      pass

  #notify all observers
  def notify(self, modifier=None):
    for observer in self._observers:
      if modifier != observer:
        observer.update(self)

  

#called if data is not understood
class FormatError(Exception):
  def __init__(self, type):
    self.expr = "unexspected format Exception"
    self.msg = "unexspected format for "+type
    
  
##used to define a segment of the skeleton
class segment:
  def __init__(self,data, quaternion=True):
    EULER_SEG_SIZE = 28
    QUATERNION_SEG_SIZE = 32
    self.is_quaternion =quaternion
    if(self.is_quaternion):
      da = data[:QUATERNION_SEG_SIZE]
      self.id , self.x , self.y , self.z , self.re , self.i , self.j , self.k= unpack('>ifffffff',data[:QUATERNION_SEG_SIZE])
      self.position = (self.x , self.y , self.z)
      self.quat = ( self.i  , self.j, self.k , self.re )
    else:
      self.id , self.x , self.y , self.z , self.rx , self.ry , self.rz = unpack('>iffffff',data[:EULER_SEG_SIZE])
      self.position(self.x , self.y , self.z)
      self.euler(self.rx , self.ry , self.rz)
      
  def print_segment(self):
    print "id:\t{0}\n".format(get_segment_name(self.id))
    print "quaternion:\t{0}\n".format(self.is_quaternion)
    print "position:\t{0}\n".format(self.position)
    if(self.is_quaternion):
      print "rotation:\t{0}\n".format(self.quat)
    else:
      print "rotation:\t{0}\n".format(self.euler)
      

##used to define an message Header from mvn mvn messages      
class xsens_header:
  
  def __init__(self,input):
    #data = unpack('>6sibbib7s','\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00')
    #data = unpack('>6sibbib7s',input[:23])
    self.id , self.sample_counter , self.datagramm_counter , self.number_of_items , self.time_code , self.avatar_id , self.free = unpack('>6sibbib7s',input[:24])

  def tmp_back(self,data):
    self.id = data[0:5]
    self.sample_counter = data[6:9]
    self.datagramm_counter = data[10]
    self.number_of_items = data[11]
    self.time_code = data[12:15]
    self.avatar_id = data[16]
    self.free = data[17:23]
    
  def print_header(self):
    print 'id:\t\t{0}\n'.format(self.id)
    print 'sample:\t{0}\n'.format(self.sample_counter)
    print 'datagramms:\t{0}\n'.format(self.datagramm_counter)
    print 'items:\t\t{0}\n'.format(self.number_of_items)
    print 'time:\t\t{0}\n'.format(self.time_code)
    print 'avatar_id:\t\t{0}\n'.format(self.avatar_id)
      
  def isQuaternion(self):
    return self.id[-2:] == '02'

  def isEuler(self):
    return self.id[-2:] == '01'

  def isPointData(self):
    return self.id[-2:] == '03'
  def isMotionGridData(self):
    return self.id[-2:] == '04'
  def isScaleInfo(self):
    return self.id[-2:] == '10'
  def isPropInfo(self):
    return self.id[-2:] == '11'
  def isMetaData(self):
    return self.id[-2:] == '12'
    
  def isSingleDatagram(self):
    return self.datagramm_counter != 0x80
  
##this class implements a thread that is reading incomming udp messages  
class xsens_client(Subject,threading.Thread):
  def __init__(self,ip='127.0.0.1',port=9763):
    #super(Subject,self).__init__()
    Subject.__init__(self)
    threading.Thread.__init__(self)
    self.UDP_IP = ip
    self.UDP_PORT = port
    self.pending = {}
    
  ##must be called to bind the socket
  def connect(self):
    print('binding socket')
    self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    self.sock.bind((self.UDP_IP,self.UDP_PORT))
    print('done')



  def read_data(self):
      recv_data = self.sock.recv(4096)
      header = xsens_header(recv_data)
      #header.print_header()
      if(not header.isSingleDatagram()):
        print '[WARNING] joining multiple datagramms is not implemented now...\n'
        print '[WARNING] This should never happen!\n'
        print '[WARNING] Check your network settings\n'
      current_position = 25;
      segments = []
      if(header.isQuaternion()):
        self.handle_motion_data(recv_data[24:],header.number_of_items)
      if(header.isEuler()):
        self.handle_motion_data(recv_data[24:],header.number_of_items,False)
	

  ##Main Thread (if not used in Ros)
  def run(self):
    running = 1
    while running:
      self.read_data()
  
  ##used to parse incomming data    
  def handle_motion_data(self , data, number_of_items, quat=True):
    self.segments = []
    current_position = 0 
    for x in range(number_of_items):
      self.segments.append(segment(data[current_position:],quat))
      if(quat):
	current_position += 32
      else:
	current_position += 28
    self.notify(self.segments)
      
    #for s in self.segments:
      #s.print_segment()

      
def main():
  client = xsens_client(ip='',port=9764)
  client.connect()
  threads.append(client)
  client.start()

##TAKE CARE OF THE SHUTDOWN
threads = [] 
  
def signal_handler(signal, frame):
        print 'You pressed Ctrl+C!\nterminating'
        for t in threads:
	  t.join()
        sys.exit(0)

#signal.pause()

##START IF THIS IS THE __main__ THREAD
if __name__ == "__main__":
  signal.signal(signal.SIGINT, signal_handler)
  main()
