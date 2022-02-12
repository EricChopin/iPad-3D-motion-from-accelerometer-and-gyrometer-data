####################################################################
# Author Eric Chopin (eric.chopin@wanadoo.fr), April 2021
#
# This programs uses the data collected on the sensors embedded 
# in an iPad 2 (iOS12) to compute its trajectory in 3 dimensions
# The program uses the accelerometer and gyrometer data
####################################################################

import motion
import numpy as np
import matplotlib.pyplot as plt
from time import sleep
import console
import math
from mpl_toolkits.mplot3d import Axes3D

offset=0.0
fig = plt.figure(figsize=(9,6))
ax = Axes3D(fig)
#ax.set_zlim(90,102)
# set labels
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
# set point of view, altitude and azimuthal axis
ax.view_init(204, 204)

def main():
  global offset
  console.alert('Motion Plot', 'When you tap Continue, accelerometer (motion) data will be recorded for 5 seconds.', 'Continue')
  console.clear()
  motion.start_updates()
  sleep(0.2)
  print('Capturing motion data...')
  num_samples = 5000
  data = []
  atts=[]
  atts2= []
  dt=0.001
  for i in range(num_samples):
    sleep(dt)
    g = motion.get_user_acceleration()
    att = motion.get_attitude()
    data.append(g)
    atts.append(att)
    motion.stop_updates()
    print('Capture finished, plotting...')
    w=0.0
    atts2.append([atts[0][0],atts[0][1],atts[0][2]])
    #There are sometimes some accidental discontinuities in the attitude data, 
    #the following fiw detects these accidents and corrects them
    for i in range(num_samples-1):
      if abs(atts[i][2]-atts[i-1][2])<0.2 and i>0:
        w= atts[i][2]-atts[i-1][2]
			
      if abs(atts[i][2]-atts[i+1][2])>0.2:
        offset += atts[i][2]-atts[i+1][2]+w
        print(str(i)+'-'+str(offset))
	
        atts2.append([atts[i+1][0],atts[i+1][1],atts[i+1][2]+offset])
		
		
  #attitude: 0=yaw, 1=roll ,2=pitch
  #acceleration: 0=x forward move ,1=y left ,2 = z 
  test_vector=[1.0,0.0,0.0] #for debug purposes
  vect=[]
  speed=[]
  speed.append([0.0,0.0,0.0])
  pos=[]
  pos.append([0.0,0.0,0.0])
  #computing the coordinates of the acceleration vector in the rest frame
  for i in range(num_samples):
    roll = atts2[i][1]
    pitch = atts2[i][0]
    yaw=atts2[i][2]
    rollmat = np.array([[1,0,0],[0,math.cos(roll),math.sin(roll)],[0,-math.sin(roll),math.cos(roll)]])
    pitchmat = np.array([[math.cos(pitch),0,math.sin(pitch)],[0,1,0],[-math.sin(pitch),0,math.cos(pitch)]])
    yawmat=np.array([[math.cos(yaw),math.sin(yaw),0],[-math.sin(yaw),math.cos(yaw),0],[0,0,1]])
    vect.append(np.dot(rollmat,np.dot(pitchmat,np.dot(yawmat,data[i]))))
		
    if i>0:
      speed.append([speed[i-1][0]+vect[i-1][0]*dt*9.8,speed[i-1][1]+vect[i-1][1]*dt*9.8,speed[i-1][2]+vect[i-1][2]*dt*9.8])
      pos.append([ pos[i-1][0]+speed[i-1][0]*dt,pos[i-1][1]+speed[i-1][1]*dt,pos[i-1][2]+speed[i-1][2]*dt])
		
  pos2 = np.transpose(pos)
	

  ax.plot(pos2[0],pos2[1],-pos2[2],'-b')
  plt.show()
	
	

if __name__ == '__main__':
	main()
