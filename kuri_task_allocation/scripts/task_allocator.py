#!/usr/bin/env python

import pylab

from mpl_toolkits.mplot3d import Axes3D
import math
import actionlib
import rospy
from std_msgs.msg import String
import random
import numpy
import yaml
import matplotlib.pyplot as plt
import ast
import geometry_msgs
from kuri_msgs.msg import *
from geometry_msgs.msg import Pose
from mavros import setpoint as SP
import rospkg


def callbackLogic(data, uav1Pose, uav2Pose, uav3Pose):
      
    def printing(seq):
        i = 0
        for obj in seq:
            i+=1
        print "The size of the object is ", i

    def printingUAVnames(seq):
        for obj in seq:
            print obj[1]


    # Just to make picking random colors and types easier
    WeightOfObject = 1  # Later below I will replace this with the score of the object to calculated weighted distances
    Color = ['RED', 'GREEN', 'BLUE', 'YELLOW', 'ORANGE'] ######$$$$$$$ The colours scoring
    Type = ['Large', 'Small', 'Accelerating']

    # Extract the content I want from the Kuri-message format and insert it into a global array
    def deconstructKuriMsg(kuridata):

        for obj in kuridata.objects:
            location_obj = numpy.array(
                (obj.pose.pose.position.x, obj.pose.pose.position.y, obj.pose.pose.position.z))
	    
            list_a.append(make_targets(obj.header.seq, location_obj, obj.color, obj.width, obj.height))
            # list_b = [obj.width]

    # Create the messages to be published
    def reconstructKuriMsg(listofobjects):
        finaloutput = Tasks()
        # print listofobjects
        for obj in listofobjects:
            drone = obj[1]
            objfortask = obj[3]
            droneid = ""

            output = Task()
            output.object = objfortask
            if(uav_name == "UAV1"):
	      output.uav_id = 1
	    elif (uav_name == "UAV2"):
	      output.uav_id = 2
	    elif (uav_name == "UAV3"):
	      output.uav_id = 3
            output.uav_name = drone
            output.task_type = ""
	    print(" task --> uav:%s, object location: %f,%f, weighted distance: %f" % (obj[1],obj[3].pose.pose.position.x, obj[3].pose.pose.position.y, obj[0]) )
            finaloutput.tasks.append(output)
        # print finaloutput
        return finaloutput


    def clearedset(seq):
        seen = set()
        seen_add = seen.add
        finaloutput = []

        for x in seq:
            loc = x[3].pose.pose.position.x+x[3].pose.pose.position.y+x[3].pose.pose.position.z
            if not loc in seen:
                seen_add(loc)
                finaloutput.append(x)
        return finaloutput

        return [x for x in seq if not (x[3].pose.pose.position.x and x[3].pose.pose.position.y in
                                       seen or seen_add(
            x[3].pose.pose.position.x and x[3].pose.pose.position.y))]


    def clearedset2(seq):
        seen = set()
        finaloutput = Tasks()
        seen_add = seen.add
        for x in seq.tasks:
            if not x.uav_name in seen:
                seen_add(x.uav_name)
                finaloutput.tasks.append(x)
        return finaloutput

        return [x for x in seq.tasks if not (x.uav_name in seen or seen_add(x.uav_name))]


    def clearedset4(seq):
        seen = set()
        finaloutput = []
        seen_add = seen.add
        for x in seq:
            if not x[1] in seen:
                seen_add(x[1])
                finaloutput.append(x)
        return finaloutput

        return [x for x in seq.tasks if not (x.uav_name in seen or seen_add(x.uav_name))]



    def clearedset5(seq):
        seen = set()
        seen2 = set()
        finaloutput = []
        seen_add = seen.add
        seen2_add = seen2.add
        for x in seq:
            loc = x[3].pose.pose.position.x + x[3].pose.pose.position.y + x[3].pose.pose.position.z
            if not x[1] in seen and loc not in seen2:
                seen_add(x[1])
                seen2_add(loc)
                finaloutput.append(x)
        return finaloutput


    # Get the scores from the YAML file
    def getColorScores():
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        packagePath = rospack.get_path('kuri_task_allocation')
        # Getting the color scores from a file, I don't use this effectivly yet
        rospack = rospkg.RosPack()
        
        with open(rospack.get_path('kuri_task_allocation')+"/config/weight.yaml", 'r') as stream:
            try:
                scores = yaml.load(stream)
                print scores
            except yaml.YAMLError as exc:
                print(exc)
        return scores

    # For test purposes, I had made this to get item locations from a file
    def getTestObjects():
        # Opening the object locations file
        with open("objectlocation.yaml", 'r') as stream2:
            try:
                items = yaml.load(stream2)
                # print type(items)
            except yaml.YAMLError as exc:
                print(exc)
        return items

    def getMovingObjectChance(objec, objDist):
        UAVspeed = 22.2222
        objSpeed = 1.38889
        CamersScanRadius = 40
        timetoObject = objDist / UAVspeed

        # Use this to compute the object speed but for now I will assume the worse case scenario
        a = math.sqrt(obj.velocity.linear.x * obj.velocity.linear.x + obj.velocity.linear.y * obj.velocity.linear.y +
                      obj.velocity.linear.z * obj.velocity.linear.z)

        PossibleObjDist = objSpeed * timetoObject

        print "Object can move for ", PossibleObjDist

        PossibleObjDist = PossibleObjDist + objDist

        if PossibleObjDist < CamersScanRadius:
            return PossibleObjDist
        else:
            return 100000

    # It would be nice to compute the total weight of the entire objects here and use it in the computation later on
    # So I would count how many R, G, B objects I have + large ones and add them up basically. This would help in scoring
    # indevedual objects

    total_score = ''

    # We can justify doing it this way since we only have three UAVs. These will hold the ranking of targets to each UAV
    rank1 = []
    rank2 = []
    rank3 = []
    rank4 = []  # for distance of objects from goal

    # UAV locations were random but now I set them to what I expect to be the general shape of the setup
    # locationOfUAVs  #use this to put the data in
    
    UAV1 = numpy.array((uav1Pose.position.x, uav1Pose.position.y, uav1Pose.position.z))  
    UAV2 = numpy.array((uav2Pose.position.x, uav2Pose.position.y, uav2Pose.position.z))
    UAV3 = numpy.array((uav3Pose.position.x, uav3Pose.position.y, uav3Pose.position.z))
    print("*******************************************************************")
    print("uav1 location x:%f , y:%f, z:%f" % (uav1Pose.position.x,uav1Pose.position.y,uav1Pose.position.z))
    print("uav2 location x:%f , y:%f, z:%f" % (uav2Pose.position.x,uav2Pose.position.y,uav2Pose.position.z))
    print("uav3 location x:%f , y:%f, z:%f" % (uav3Pose.position.x,uav3Pose.position.y,uav3Pose.position.z))
    print("*******************************************************************")

    #not used variable
    GOAL = numpy.array((0, 0, 100))

    # Target object, contains an ID, location, color and a type
    class Target(object):
        number = 0
        location_obj = numpy.array(())
        color = ""
        # size = "" for now I will do it in type
        type = ""
        width = ""
        height = ""

        # The class "constructor" - It's actually an initializer
        def __init__(self, number, location_obj, color, width, height):
            self.number = number
            self.location_obj = location_obj
            self.color = color
            # self.size = size
            self.width = width
            self.height = height

    def make_targets(number, location_obj, color, width, height):
        target = Target(number, location_obj, color, width, height)
        return target

        # Getting the locations from the Yaml file

    loop1 = 0  # I want only 20 objects so I made an itirator my self
    indexofyaml = 1  # the object id in the yaml file, TODO: I should probably change the target IDs in the yaml file to 0 etc
    list_a = []
    xLocations = []  # Will use these to plot
    yLocations = []
    zLocations = []
    Locations = []
    colours = []   # From what I remember, this is for the graphing
    area = []      # From what I remember, this is for the graphing



    # Compute distance between UAVs and objects, only three UAVS, can justify copy pasting the code
    loop2 = 0
    loopUAV = 0
    scores = getColorScores()
    # print scores





    #### Important :: I can optimise this by having the distances all calculated in one loop. no need to reiterate the list.
    #### However, this gives me more control


    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> uncomment if you want UAV 1 (Explorer) to have a task
    #removed in order to allocate only for two drones
    """ <<<<<<< Computing 2d distance of all objects to UAV1 >>>>>>>"""
    #print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ")    
    #for obj in data.objects:
      #if not (obj.color == "ORANGE") and not (obj.color == "YELLOW"):
	#print("object color: %s" % (obj.color))      
        #location_obj = numpy.array(
            #(obj.pose.pose.position.x, obj.pose.pose.position.y, obj.pose.pose.position.z))
        #Locations.append(location_obj)
        #colours.append(obj.color)
        #area.append(10)

        #if obj.velocity.linear.x != 0 or obj.velocity.linear.y != 0 or obj.velocity.linear.z != 0:
            #print "object is in motion"
            #print "current distance of UAV1 from object is ", dist
            #print "current x ", obj.pose.pose.position.x
            #print "current y ", obj.pose.pose.position.y

            #dist = numpy.linalg.norm(UAV1 - location_obj)
            #possibleObjectLocation = getMovingObjectChance(obj, dist)
            #dist = possibleObjectLocation + dist


        #else:
            #dist = numpy.linalg.norm(UAV1 - location_obj)
            #print "object is stationary"
            #print "distance of UAV1 from object is ", dist
            #print "x ", obj.pose.pose.position.x
            #print "y ", obj.pose.pose.position.y

        #if obj.color != 'Nan':
            #WeightOfObject = scores.get(obj.color)
	    #print "weight of object ", WeightOfObject

        #weighteddist = dist / WeightOfObject
        #print "weighted distance of the object = ", weighteddist
        #rank1.append((weighteddist, 'UAV1', obj.header.seq, obj))
        #rank1.sort()
        #print "UAV one ranking", rank1
        #print "               "



    """ <<<<<<<  Computing 2d distance of all objects to UAV2 >>>>>>>"""
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ")
    for obj in data.objects:
      if not (obj.color == "ORANGE") and not (obj.color == "YELLOW"):
	print("object color: %s" % (obj.color))     
 
        location_obj = numpy.array(
            (obj.pose.pose.position.x, obj.pose.pose.position.y, obj.pose.pose.position.z))

        if obj.velocity.linear.x != 0 or obj.velocity.linear.y != 0 or obj.velocity.linear.z != 0:
            print "object is in motion"
            print "current distance of UAV2 from object is ", dist
            print "current x ", obj.pose.pose.position.x
            print "current y ", obj.pose.pose.position.y

            dist = numpy.linalg.norm(UAV2 - location_obj)
            possibleObjectLocation = getMovingObjectChance(obj, dist)
            dist = possibleObjectLocation + dist


        else:
            dist = numpy.linalg.norm(UAV2 - location_obj)
            print "object is stationary"
            print "distance of UAV2 from object is ", dist
            print "x ", obj.pose.pose.position.x
            print "y ", obj.pose.pose.position.y

        if obj.color != 'Nan':
            WeightOfObject = scores.get(obj.color)
	    print "weight of object ", WeightOfObject
	    
        weighteddist = dist / WeightOfObject
        print "weighted distance of the object = ", weighteddist
        rank2.append((weighteddist, 'UAV2', obj.header.seq, obj))
        rank2.sort()
        # print "UAV two ranking", rank2
        # print "               "



    """ <<<<<<<  Computing 2d distance of all objects to UAV3 >>>>>>>"""
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ")
    for obj in data.objects:
      if not (obj.color == "ORANGE") and not (obj.color == "YELLOW"):
	print("object color: %s" % (obj.color))      
        location_obj = numpy.array(
            (obj.pose.pose.position.x, obj.pose.pose.position.y, obj.pose.pose.position.z))

        if obj.velocity.linear.x != 0 or obj.velocity.linear.y != 0 or obj.velocity.linear.z != 0:
            print "object is in motion"
            print "current distance of UAV3 from object is ", dist
            print "current x ", obj.pose.pose.position.x
            print "current y ", obj.pose.pose.position.y

            dist = numpy.linalg.norm(UAV3 - location_obj)
            possibleObjectLocation = getMovingObjectChance(obj, dist)
            dist = possibleObjectLocation


        else:
            dist = numpy.linalg.norm(UAV3 - location_obj)
            print "object is stationary"
            print "distance of UAV3 from object is ", dist
            print "x ", obj.pose.pose.position.x
            print "y ", obj.pose.pose.position.y

        if obj.color != 'Nan':
            WeightOfObject = scores.get(obj.color)
	    print "weight of object ", WeightOfObject
	    
        weighteddist = dist / WeightOfObject
        print "weighted distance of the object = ", weighteddist        
        rank3.append((weighteddist, 'UAV3', obj.header.seq, obj))
        rank3.sort()
        # print "UAV three ranking", rank3
        # print "                 "



    FinalRanking = []
    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> uncomment if you want three tasks for the three UAVs
    #FinalRanking = rank1 + rank2 + rank3 #removed rank 1 since we need to to allocate only for uav2, and uav3
    FinalRanking =  rank2 + rank3 
    
    FinalRanking.sort()
    FinalRanking2 = clearedset5(FinalRanking)
    #FinalRanking2 = clearedset4(FinalRanking2)
    #FinalRanking2 = clearedset(FinalRanking)
    
    print(">>>>>>>>>>> NAVIGATION Tasks %f <<<<<<<<<<<" % (len(FinalRanking2)))
    NavigationTasks = reconstructKuriMsg(FinalRanking2)


    ## not sure if they are important ( ?? remove in future )
    #NavigationTasks = clearedset2(NavigationTasks)
    #print NavigationTasks
    #FinalRanking2 = clearedset(NavigationTasks)


    # Removes all the other entries except for the closes object/UAV pair
    # This removes duplicate object IDs from the list so that no 2 UAVs go to the same object
    # Deletes the entry which contains the object id in question from the list given

    def removeObject(list, idofentry):

        list2 = []
        for x in list:
            if x[1] == idofentry:
                print x

    """ <<<<<<<<<< PLOTTING the tasks >>>>>>>>>>"""
    for obj in FinalRanking2:
      location_obj = numpy.array(
            (obj[3].pose.pose.position.x, obj[3].pose.pose.position.y, obj[3].pose.pose.position.z))      
      Locations.append(location_obj)
      colours.append(obj[3].color)
      area.append(10)
      
    Locations.append(UAV1)
    Locations.append(UAV2)
    Locations.append(UAV3)

    x = []
    y = []
    z = []

    for obj in Locations:
        x.append(obj[0])
        y.append(obj[1])
        z.append(obj[2])


    colours.append("y")  # now the 21th item is UAV1
    colours.append('c')  # now the 22th item is UAV2
    colours.append('k')  # now the 23rd item is UAV3

    area.append(70)  # setting the size of the drone
    area.append(70)
    area.append(70)

    def onpick(event):
        ind = event.ind
        print 'onpick3 scatter:', ind, numpy.take(x, ind), numpy.take(y, ind)

    plt.close("all")
    fig = plt.figure()
    fig2 = plt.figure()
    
    # fig.canvas.mpl_connect('pick_event', onpick)
    UAVx = [x[-3], x[-2], x[-1]]
    UAVy = [y[-3], y[-2], y[-1]]
    UAVz = [z[-3], z[-2], z[-1]]



    ax3 = fig2.add_subplot(111, projection = '3d')
    ax2 = fig.add_subplot(111)


    # Ultra lazy mode, fix this later..
    # Basically just graphing the UAV going to one of these points. Check which UAV has the task and plot a line
    
    # First task
    if NavigationTasks.tasks[0].uav_name == "UAV1":
        xx = [UAVx[0], NavigationTasks.tasks[0].object.pose.pose.position.x]
        yy = [UAVy[0], NavigationTasks.tasks[0].object.pose.pose.position.y]
        zz = [UAVz[0], NavigationTasks.tasks[0].object.pose.pose.position.z]
        ax2.plot(xx, yy, '-y')
        ax3.plot(xx, yy, zz, c = 'y')


    if NavigationTasks.tasks[0].uav_name == "UAV2":
        xx = [UAVx[1], NavigationTasks.tasks[0].object.pose.pose.position.x]
        yy = [UAVy[1], NavigationTasks.tasks[0].object.pose.pose.position.y]
        zz = [UAVz[1], NavigationTasks.tasks[0].object.pose.pose.position.z]

        ax2.plot(xx, yy, '-c')
        ax3.plot(xx, yy, zz, c = 'c')

    if NavigationTasks.tasks[0].uav_name == "UAV3":
        xx = [UAVx[2], NavigationTasks.tasks[0].object.pose.pose.position.x]
        yy = [UAVy[2], NavigationTasks.tasks[0].object.pose.pose.position.y]
        zz = [UAVz[2], NavigationTasks.tasks[0].object.pose.pose.position.z]

        ax2.plot(xx, yy, '-k')
        ax3.plot(xx, yy, zz, c = 'k')

    # Second Task
    if NavigationTasks.tasks[1].uav_name == "UAV1":
        xx = [UAVx[0], NavigationTasks.tasks[1].object.pose.pose.position.x]
        yy = [UAVy[0], NavigationTasks.tasks[1].object.pose.pose.position.y]
        ax2.plot(xx, yy, '-y')
        ax3.plot(xx, yy, zz)


    if NavigationTasks.tasks[1].uav_name == "UAV2":
        xx = [UAVx[1], NavigationTasks.tasks[1].object.pose.pose.position.x]
        yy = [UAVy[1], NavigationTasks.tasks[1].object.pose.pose.position.y]
        ax2.plot(xx, yy, '-c')
        ax3.plot(xx, yy, zz, c = "c")


    if NavigationTasks.tasks[1].uav_name == "UAV3":
        xx = [UAVx[2], NavigationTasks.tasks[1].object.pose.pose.position.x]
        yy = [UAVy[2], NavigationTasks.tasks[1].object.pose.pose.position.y]
        ax2.plot(xx, yy, '-k')
        ax3.plot(xx, yy, zz, c = "k")

    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> uncomment if you want three tasks for the three UAVs
    # Third task
    #if NavigationTasks.tasks[2].uav_name == "UAV1":
        #xx = [UAVx[0], NavigationTasks.tasks[2].object.pose.pose.position.x]
        #yy = [UAVy[0], NavigationTasks.tasks[2].object.pose.pose.position.y]
        #ax2.plot(xx, yy, '-y')
        #ax3.plot(xx, yy, zz, c = "y")


    #if NavigationTasks.tasks[2].uav_name == "UAV2":
        #xx = [UAVx[1], NavigationTasks.tasks[2].object.pose.pose.position.x]
        #yy = [UAVy[1], NavigationTasks.tasks[2].object.pose.pose.position.y]
        #ax2.plot(xx, yy, '-c')
        #ax3.plot(xx, yy, zz, c = "c")


    #if NavigationTasks.tasks[2].uav_name == "UAV3":
        #xx = [UAVx[2], NavigationTasks.tasks[2].object.pose.pose.position.x]
        #yy = [UAVy[2], NavigationTasks.tasks[2].object.pose.pose.position.y]
        #ax2.plot(xx, yy, '-k')
        #ax3.plot(xx, yy, zz, c = "k")


    labels = []
    i=0
    while (i < len(FinalRanking2) ):
        labels.append("Object "+str(colours[i]) +"[ "+str(Locations[i])+" ]" )
        i += 1

    labels.append("UAV1")
    labels.append("UAV2")
    labels.append("UAV3")


    ax1 = fig.add_subplot(111)
    ii = 0

    for label in labels:
                ax1.annotate(
                    label,
                    xy=(x[ii], y[ii]), xytext=(-20, 20),
                    textcoords='offset points', ha='right', va='bottom',
                    bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.5),
                    arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
                ii+=1

    col = ax1.scatter(x, y, area, colours, picker=True)
    fig.show()
    col = ax3.scatter(x, y, z, c = colours, s=area, picker=True, marker="o")
    fig2.show()
    
    """ <<<<<<<<<<<< End of plotting part >>>>>>>>>>"""
     
    return NavigationTasks

