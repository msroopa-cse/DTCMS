#!/usr/bin/env python
"""
@file    runner.py
@author  Roopa

@date    2017-03-26
@version $Id: runner.py  $

Tutorial for traffic light control via the TraCI interface.
This scenario models a pedestrian crossing which switches on demand.

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2009-2015 DLR/TS, Germany

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
import sys
import optparse
import subprocess
import random
import math
SUMO_HOME = "C:/Users/shah/Desktop/SIoVProject/CodesFolder"

# the directory in which this script resides
THISDIR = os.path.dirname(__file__)


# we need to import python modules from the $SUMO_HOME/tools directory
# If the the environment variable SUMO_HOME is not set, try to locate the python
# modules relative to this script
try:
    # tutorial in tests
    sys.path.append(os.path.join(THISDIR, '..', '..', '..', '..', "tools"))
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        THISDIR, "..", "..", "..")), "tools"))  # tutorial in docs
    import sumolib
    import traci
    from sumolib import checkBinary
    import randomTrips
   
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")
import traci.constants as tc

# the port used for communicating with your sumo instance
PORT = 8206
print(dir(traci.lane))
print(dir(traci.vehicle))
#print help(traci.vehicle.getBestLanes)
#print help(traci.vehicle.getAngle)
left = [['E10','E04'],['E20','E01'],['E30','E02'],['E40','E03']]
straight = [['E10','E03'],['E20','E04'],['E30','E01'],['E40','E02']]
right = [['E10','E02'],['E20','E03'],['E30','E04'],['E40','E01']]

list_of_max=[]
veh_no=[]
def run():
    """execute the TraCI control loop"""
   
    traci.init(PORT)
    while traci.simulation.getMinExpectedNumber() > 0:
     #A=np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0]])
     #d={}
     column_names = ['1','2','3','4']
     row_names    = ['_2', '_1', '_0']

     matrix = np.reshape((0, 0, 0, 0, 0, 0, 0, 0, 0 , 0 , 0 , 0), (3, 4))
     df = pd.DataFrame(matrix, columns=column_names, index=row_names)
     c_names = ['2', '1', '0']
     r_names = ['RS1','RS2','RS3','RS4']

     matrix1 = np.reshape((0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0), (4, 3))
     df1 = pd.DataFrame(matrix1, columns=c_names, index=r_names)
     
  
     
     traci.simulationStep()
    
     
     l=traci.lane.getIDList()
    
     RS1= l[34:37]
     RS2= l[37:40]
     RS3= l[40:43]
     RS4= l[43:46]
     RS= [RS1,RS2,RS3,RS4]
     for segment in RS:

       
       for lane in segment:
         veh_list = traci.lane.getLastStepVehicleIDs(lane)
         #print veh_list , lane 
         le=rig=stra=0
         for vehi in veh_list:
             rou = traci.vehicle.getRoute(vehi)
             if rou in left:
                 le = le + 1
             elif rou in right:
                 rig = rig + 1
             elif rou in straight:
                 stra = stra +1
         print(le , rig , stra , lane)
         if '_2' in lane:
             
             if stra==0:
                   

                     df['3']['_2']=1
                     
                     df['2']['_1']=1
                     df['2']['_0']=1
                         
                     
             elif rig==0:
                 if stra!=0:
                     if le==0:
                             df['2']['_2']=1
                     
                             df['2']['_1']=1
                             df['2']['_0']=1
                          
             elif stra!=0:
                 if le!=0:
                     df['4']['_2']=1
                     
                     df['4']['_1']=1
                     df['4']['_0']=1
         elif '_0' in lane:
             if stra==0:
                 df['1']['_0']=1
     print("******The Flow attribute Matrix is as follows******")
     print(df)
     #.plot()
     for segment in range(0,4):

       
       for lane in RS[segment]:
         

	     #finding the number of vehiles
         n = traci.lane.getLastStepVehicleNumber(lane)
         lane_=lane[4:5]
         # Declare speed, acceleration, distnace and passing rate variables
         speed,accel,distance,ptime,prate=0.0,0.0,0.0,0.0,0.0
         #getting the vehicle ids
         veh_list = traci.lane.getLastStepVehicleIDs(lane)
         #get last and second last vehicle ids
         #print("+++++++++++++++++")
         
         #print("+++++++++++++++++")

         # get the distance of the last vehicle from the intersection
         if not veh_list : print("*********** veh_list is empty to show distance from junction")
         else:
           #print("+++++++++++++++++")
           distance=traci.vehicle.getDistance(veh_list[-1])
           print("Distance from junction is "+str(distance))
           #print ("+++++++++++++++")

         # get speed of last vehicle
         if not veh_list : print("*********** veh_list is empty to show last speed ")
         else :
             #print(" ?????????????????????????? ")
             speed=traci.vehicle.getSpeed(veh_list[-1])
             print("Speed of last vehicle is " +str(speed))
             #print(" ?????????????????????????? ")


         '''for veh in veh_list:
             var_speed=traci.vehicle.getSpeed(veh)
             print(var_speed)'''

         #get acceleration of second last vehicle
         if not veh_list: print("*********** veh_list is empty to show second last acceleration ")
         else:
             # print("###########################")
             accel= traci.vehicle.getAccel(veh_list[-1])
             print("Acceleration of second last vehicle is " +str(accel))
             #print("###########################")
             #print acc

         
         #calculate the passing rate using the formula
         if not veh_list: print("*********** veh_list is empty to show passing rate ")
         else:
            ptime = (-speed + math.sqrt((speed*speed)+2*accel*distance))/accel
            print ("Passing time is "+str(ptime))
            
            
            
            ##########################################enter passing time matrix here
            #col = ['2', '1', '0']
            #row = ['RS1','RS2','RS3','RS4']

            #matrix2 = np.reshape((0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0), (4, 3))
            #df2 = pd.DataFrame(matrix2, columns=col, index=row)
            
            
            
            if ptime!=0:
                prate = n/ptime
                
            print ("Flow rate is "+str(prate))
            a=(str(lane_))
            #print a
            b=('RS'+str(segment+1))
            #print b
            df1[a][b]=prate
            #df2[a][b]=ptime
     print("******The Flow rate Matrix is as follows******")
     print(df1)
     #df1.plot() 
                
     X = [[0,0,0],
          [0,0,0],
          [0,0,0],
          [0,0,0]]
       # 3x4 matrix
     Y = [[0,0,0,0],
         [0,0,0,0],
         [0,0,0,0]]
       # result is 3x4
     APR = [[0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [0,0,0,0]]

      # iterate through rows of X
     for i in range(len(X)):
        # iterate through columns of Y
        for j in range(len(Y[0])):
            # iterate through rows of Y
            for k in range(len(Y)):
                APR[i][j] += X[i][k] * Y[k][j]

     #for r in APR:
      #  print(r)          
                
     print("******The aggregated flow rate is as follows******")
     for r in APR:
        print(APR)
     print("******The 16 Condition Matrices Printed below******")
     data1= [[1,0,0,0],[1,0,0,0],[0,0,1,0],[0,0,1,0]]
     TM1 = pd.DataFrame(data1,dtype=float)
     print (TM1)
     data2 = [[0,0,1,0],[0,0,1,0],[1,0,0,0],[1,0,0,0]]
     TM2 = pd.DataFrame(data2,dtype=float)
     print (TM2)
     data3 = [[0,1,0,0],[0,1,0,0],[0,0,0,0],[0,0,0,0]]
     TM3 = pd.DataFrame(data3,dtype=float)
     print (TM3)
     data4 = [[0,0,0,0],[0,0,0,0],[0,1,0,0],[0,1,0,0]]
     TM4 = pd.DataFrame(data4,dtype=float)
     print (TM4)
     data5 = [[0,0,0,1],[0,0,0,0],[1,0,0,0],[0,0,0,0]]
     TM5 = pd.DataFrame(data5,dtype=float)
     print (TM5)
     data6 = [[0,0,0,0],[0,0,0,1],[0,0,0,0],[1,0,0,0]]
     TM6 = pd.DataFrame(data6,dtype=float)
     print (TM6)
     data7 = [[0,0,0,0],[1,0,0,0],[0,0,0,1],[0,0,0,0]]
     TM7 = pd.DataFrame(data7,dtype=float)
     print (TM7)
     data8 = [[1,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,1]]
     TM8 = pd.DataFrame(data8,dtype=float)
     print (TM8)
     data9 = [[1,0,0,0],[0,1,0,0],[0,0,0,0],[1,0,0,0]]
     TM9 = pd.DataFrame(data9,dtype=float)
     print (TM9)
     data10 = [[0,1,0,0],[1,0,0,0],[1,0,0,0],[0,0,0,0]]
     TM10 = pd.DataFrame(data10,dtype=float)
     print (TM10)
     data11 = [[0,0,0,0],[1,0,0,0],[0,1,0,0],[1,0,0,0]]
     TM11 = pd.DataFrame(data11,dtype=float)
     print (TM11)
     data12 = [[1,0,0,0],[0,0,0,0],[1,0,0,0],[0,1,0,0]]
     TM12 = pd.DataFrame(data12,dtype=float)
     print (TM12)
     data13 = [[1,0,1,0],[0,0,0,0],[1,0,0,0],[1,0,0,0]]
     TM13 = pd.DataFrame(data13,dtype=float)
     print (TM13)
     data14 = [[0,0,0,0],[1,0,1,0],[1,0,0,0],[1,0,0,0]]
     TM14 = pd.DataFrame(data14,dtype=float)
     print (TM14)
     data15 = [[1,0,0,0],[1,0,0,0],[1,0,1,0],[0,0,0,0]]
     TM15 = pd.DataFrame(data15,dtype=float)
     print (TM15)
     data16 = [[1,0,0,0],[1,0,0,0],[0,0,0,0],[1,0,1,0]]
     TM16 = pd.DataFrame(data16,dtype=float)
     print (TM16)
     #enter code here
     list_T= [TM1,TM2,TM3,TM4,TM5,TM6,TM7,TM8,TM9,TM10,TM11,TM12,TM13,TM14,TM15,TM16]
     WAPR=[]
     STOTAL=[]
     for frame in list_T:
            
            #WAPR.append(np.matmul(APR,frame))
            WAPR.append(APR*frame)
     
     #print WAPR
     for frame in WAPR:
            STOTAL.append(frame.sum().sum())
     print("******The following list contains the sum of flow rates for each ont f the 16 condition matrices when multiplied with aggregated flow rates******")
     print(STOTAL)
   
     #plt.rcdefaults()
     
 
     #objects = ('TM1', 'TM2', 'TM3', 'TM4' , 'TM5' , 'TM6' , 'TM7' , 'TM8' , 'TM9' , 'TM10' , 'TM11' , 'TM12' , 'TM13' , 'TM14' , 'TM15' , 'TM16')
     #y_pos = np.arange(len(objects))
     #performance = STOTAL
 
     #plt.bar(y_pos, performance, align='center', alpha=0.5)
     #plt.xticks(y_pos, objects)
     #plt.ylabel('Total Passing Rate')
     #plt.title('Transition Matrix Selection')
 
     #plt.show()

     max_value = max(STOTAL)
     
     print(max_value)
     matrix_selection = STOTAL.index(max_value)
     #print matrix_selection
     print("#####The selected Condition matrix#####")
     print (list_T[matrix_selection])
     no_of_veh = traci.vehicle.getIDCount()

     print(no_of_veh)

     tgreen = no_of_veh/max_value
     
     if tgreen<15 :
          tgreen=15
     elif tgreen>150:
          tgreen=150
     print("The Tgreen value after applying limits is as given below")
     print(tgreen)
     
     #t=traci.simulation.getCurrentTime()
     #list_of_max.append(max_value)
     #print list_of_max
     #print max(list_of_max)
      
     
     #print help(traci.trafficlights)
    # print(traci.trafficlights.getIDList())
     
    #traci.trafficlights.setRedYellowGreenState('0','rRGGGG')
    #traci.trafficlights.setPhaseDuration('0',tgreen)
     #print traci.trafficlights.getControlledLanes('0')
     #print "plzzz work" 
     #print traci.trafficlights.getCompleteRedYellowGreenDefinition('0')
     #print "plzzz work"
     #print traci.trafficlights.getControlledLinks('0')
     #print "plzzz work" 
     #print traci.trafficlights.getNextSwitch('0')
     #print "plzzz work" 
     #print traci.trafficlights.getPhase('0')
     #print "plzzz work" 
     #print traci.trafficlights.getPhaseDuration('0')
     #print "plzzz work" 
     #print traci.trafficlights.getProgram('0')
     #print "plzzz work" 
     #print traci.trafficlights.getRedYellowGreenState('0')
     #print "plzzz work" 
     #traci.trafficlights.setLinkState('0','E10_0', 'E02_0', ':0_0_0','rRgGyYoOu') 
     

    
     
    
     
     
     
 
     
     
     
          
            
     
     
                      
                   
     
   
                       
 #postioning of the dictionary think about it    
     
       
       
    sys.stdout.flush()
    traci.close()


   
    


# this is the main entry point of this script
if __name__ == "__main__":
    # load whether to run with or without GUI
   

    # this script has been called from the command line. It will start sumo as a
    
    sumoBinary = checkBinary('sumo-gui')

    

  

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    sumoProcess = subprocess.Popen([sumoBinary,
                                    '-c', os.path.join('hello.sumocfg'),
                                    '--remote-port', str(PORT)],
                                   stdout=sys.stdout, stderr=sys.stderr)
    run()
    sumoProcess.wait()
