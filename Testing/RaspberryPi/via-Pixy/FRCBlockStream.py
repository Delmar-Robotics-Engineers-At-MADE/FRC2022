#This program streams block data from a pixy2 to pynetworktables in order to communicate with a ROBORIO, intended to track yellow energy orbs, It takes the server IP as a parameter
# based on https://github.com/charmedlabs/pixy2/blob/master/src/host/libpixyusb2_examples/python_demos/get_blocks_python_demo.py
from __future__ import print_function
#import pixy

import sys
import time
import logging
import threading

#had trouble finding pynetworktables module in early 2020, but not with later 2020 FRCVision image, was...
#sys.path.append('/home/pi/.local/lib/python3.7/dist-packages/pynetworktables-2019.0.1-py3.7.egg')

from networktables import NetworkTables

from ctypes import *
import pixy
from pixy import *

#Listener and vars for verifying connection to networktable
cond = threading.Condition()
notified= [False]

def connectionListener(connected, info):
  print(info, '; Connected=%s' % connected)
  with cond:
    notified[0] = True
    cond.notify()

ip = '10.80.77.2'
#use command line argument for ip, if it exists
if len(sys.argv) == 2:
	ip = sys.argv[1]
    # was if != 2: print("Error: specify an IP to connect to!")
    #              exit(0)

#Connecting to table at ip
NetworkTables.initialize(ip)

#Verifying table connection and waiting until connected
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)
with cond:
  print('Waiting for networktable server...')
  if not notified[0]:
    cond.wait()
print('Connected to networktable server. Preparing to stream pixy blocks data')

#Creating name of subtable to add 
pb = NetworkTables.getTable("PixyBlocks")
print('Network table created')

status = -1 # -1 means camera is not inited yet

while (status == -1):
  print('Looking for pixycam...')
  try:
    print('Init...')
    initResult = pixy.init()
    if initResult != 0:
      print('Pixy locked or not available')
    else:
      print('Change program...')
      pixy.change_prog ("color_connected_components");
      status = 0
  except:
    print('Exception accessing pixycam, sleeping then trying again...')
    pb.putNumber("STATUS",-1) # -1 means camera exception
  time.sleep(1.0) # was 0.1
print ('successfully inited pixycam')

class Blocks (Structure):
  _fields_ = [ ("m_signature", c_uint),
    ("m_x", c_uint),
    ("m_y", c_uint),
    ("m_width", c_uint),
    ("m_height", c_uint),
    ("m_angle", c_uint),
    ("m_index", c_uint),
    ("m_age", c_uint) ]

blocks = BlockArray(100)
frame = 0
minY = 10
minWidth = 100
maxWidth = 2500

while 1:
  index_of_closest = 0
  try:
    count = pixy.ccc_get_blocks (100, blocks)
  except:
    count = 0

  if count == 0:
    pb.putNumber("STATUS",0) # no targets
  else:
    pb.putNumber("STATUS",1) # 1 means targets seen
    #print('frame %3d:' % (frame))
    frame = frame + 1
    for index in range (0, count):
      width = blocks[index].m_width * blocks[index].m_height
      #print('[BLOCK: SIG=%d X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d]' % (blocks[index].m_signature, blocks[index].m_x, blocks[index].m_y, blocks[index].m_width, blocks[index].m_height))
      if (blocks[index].m_y > minY) and (blocks[index].m_age > 0) \
          and (width > minWidth) and (width < maxWidth) \
          and (blocks[index].m_y < blocks[index_of_closest].m_y):
        index_of_closest = index
      #adding values to networktable
      pb.putNumber("FRAME",frame)
      pb.putNumber("SIG",blocks[index_of_closest].m_signature)
      pb.putNumber("X",blocks[index_of_closest].m_x)
      pb.putNumber("Y",blocks[index_of_closest].m_y)
      pb.putNumber("WIDTH",blocks[index_of_closest].m_width)
      pb.putNumber("HEIGHT",blocks[index_of_closest].m_height)
      pb.putNumber("ANGLE",blocks[index_of_closest].m_angle)
      pb.putNumber("INDEX",blocks[index_of_closest].m_index)
      pb.putNumber("AGE",blocks[index_of_closest].m_age)
  time.sleep(0.1)
