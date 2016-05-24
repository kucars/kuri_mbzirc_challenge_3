#!/usr/bin/env python



#Original Author: Abdelrahman Ali AlMahmoud
#
#


import rospy
from std_msgs.msg import String
import random
import numpy
import yaml
#from matplotlib.pyplot import figure, show
import matplotlib.pyplot as plt
import ast
import geometry_msgs
from kuri_msgs.msg import *
from geometry_msgs.msg import Pose
from mavros import setpoint as SP


GlobalObjectInformationArray = []



def callback(data):

    #Just to make picking random colors and types easier
    WeightOfObject = 1  #Later below I will replace this with the score of the object to calculated weighted distances
    Color = ['R', 'G', 'B']
    Type = ['Large', 'Small', 'Accelerating']


    #Extract the content I want from the Kuri-message format and insert it into a global array
    def deconstructKuriMsg(kuridata):

        for obj in kuridata.objects:
            location_obj = numpy.array((obj.pose.pose.position.x, obj.pose.pose.position.y, obj.pose.pose.position.z))

            list_a.append(make_targets(obj.header.seq, location_obj, obj.color, obj.width, obj.height))

            #list_b = [obj.width]




    #Create the messages to be published
    def reconstructKuriMsg(listofobjects):
        finaloutput = NavTasks()
        print listofobjects
        for obj in listofobjects:
            drone = obj[1]
            objfortask = obj[3]
            output = NavTask()
            output.object = objfortask
            output.uav_name=drone
            finaloutput.tasks.append(output)
        print "3333333333333"
        print finaloutput
        return finaloutput



    def clearedset(seq):
        seen = set()
        seen_add = seen.add

        return [x for x in seq if not (x[3].pose.pose.position.x and x[3].pose.pose.position.y in
                                       seen or seen_add(x[3].pose.pose.position.x and x[3].pose.pose.position.y))]

    def clearedset2(seq):
        seen = set()
        finaloutput = NavTasks()
        seen_add = seen.add
        for x in seq.tasks:
            if not x.uav_name in seen:
                seen_add(x.uav_name)
                finaloutput.tasks.append(x)
        return finaloutput

        return [x for x in seq.tasks if not (x.uav_name in seen or seen_add(x.uav_name))]

    #Get the scores from the YAML file
    def getColorScores():
        # Getting the color scores from a file, I don't use this effectivly yet
        with open("~/catkin_ws/src/kuri_mbzirc_challenge_3/kuri_task_allocation/config/weight.yaml", 'r') as stream:
            try:
                scores = yaml.load(stream)
                # print scores
            except yaml.YAMLError as exc:
                print(exc)
        return scores




    #For test purposes, I had made this to get item locations from a file
    def getTestObjects():
        # Opening the object locations file
        with open("objectlocation.yaml", 'r') as stream2:
            try:
                items = yaml.load(stream2)
                # print type(items)
            except yaml.YAMLError as exc:
                print(exc)
        return items




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
    UAV1 = numpy.array((100, 100, 100))
    UAV2 = numpy.array((450, 800, 100))
    UAV3 = numpy.array((800, 100, 100))
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

    # print 'UAV1 location', UAV1
    # print 'UAV2 location', UAV2
    # print 'UAV3 location', UAV3



    # Scores for now are static, will read them from a file later

    # R = 10 #Red
    # G = 15 #Green
    # B = 20 #Blue


    # Compute distance between UAVs and objects, only three UAVS, can justify copy pasting the code
    loop2 = 0
    loopUAV = 0
    scores = getColorScores()
    #print scores





#### Important :: I can optimise this by having the distances all calculated in one loop. no need to reiterate the list

    for obj in data.objects:
        location_obj = numpy.array((obj.pose.pose.position.x, obj.pose.pose.position.y, obj.pose.pose.position.z))
        dist = numpy.linalg.norm(UAV1 - location_obj)
        if obj.color != 'Nan':
            WeightOfObject = scores.get(obj.color)
        weighteddist = dist / WeightOfObject
        rank1.append((weighteddist, 'UAV1', obj.header.seq, obj))
        rank1.sort()
        # print "UAV one ranking", rank1
        # print "               "

    for obj in data.objects:
        location_obj = numpy.array((obj.pose.pose.position.x, obj.pose.pose.position.y, obj.pose.pose.position.z))
        dist = numpy.linalg.norm(UAV2 - location_obj)
        if obj.color != 'Nan':
            WeightOfObject = scores.get(obj.color)
        weighteddist = dist / WeightOfObject
        rank2.append((weighteddist, 'UAV2', obj.header.seq, obj))
        rank2.sort()
        # print "UAV two ranking", rank2
        # print "               "



    for obj in data.objects:
        location_obj = numpy.array((obj.pose.pose.position.x, obj.pose.pose.position.y, obj.pose.pose.position.z))
        dist = numpy.linalg.norm(UAV3 - location_obj)
        if obj.color != 'Nan':
            WeightOfObject = scores.get(obj.color)
        weighteddist = dist / WeightOfObject
        rank3.append((weighteddist, 'UAV3', obj.header.seq, obj))
        rank3.sort()
        # print "UAV three ranking", rank3
        # print "                 "


        # print "Goal ranking   "
        # print "               "

    FinalRanking = []

    FinalRanking = rank1 + rank2 + rank3
    FinalRanking.sort()
    #print FinalRanking
    FinalRanking2 = clearedset(FinalRanking)


    NavigationTasks = reconstructKuriMsg(FinalRanking2)


    NavigationTasks = clearedset2(NavigationTasks)

   # FinalRanking2 = clearedset(NavigationTasks)



    # Removes all the other entries except for the closes object/UAV pair



        # This removes duplicate object IDs from the list so that no 2 UAVs go to the same object



        # Deletes the entry which contains the object id in question from the list given

    def removeObject(list, idofentry):

        list2 = []
        for x in list:
            if x[1] == idofentry:
                print x






    # print FinalRanking
    # #this prints all of the target rankings for each UAV, could create conflict if 2 UAVs try to pick up the same target

    #This block of code was removed for testing, it computes the distance from the goal
    '''
    hello = clearedset(FinalRanking)  # printing the final set which has priorities set to UAVs, other UAVs will not see the items
    # print hello
    hello = clearedset2(hello)
    print hello

    c = iter([0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2])  # this is to iterate the UAV IDs and put them into the closest to the goal. I did this to have the first UAV in line be assigned the next object

    for obj in list_a:

        dist = numpy.linalg.norm(GOAL - obj.location_obj)
        if obj.color != 'Nan':
            WeightOfObject = scores.get(obj.color)
            # print WeightOfObject

        firstTargets = hello[c.next()]
        # print firstTargets
        weighteddist = dist / WeightOfObject
        rank4.append((weighteddist, obj.number, firstTargets[2]))
        rank4.sort()

    rank4 = clearedset(rank4)
    # print type(rank4)
    # rank4 = clearedset2(rank4)
    # print rank4
    '''

    # which a different UAV got assigned
    # print ast.literal_eval(hello)

    pub = rospy.Publisher('kuri_msgs/NavTasks', NavTasks, queue_size=1)

    # pub1 = rospy.Publisher('uav1_target_location', Pose, queue_size=1)
    # pub2 = rospy.Publisher('uav2_target_location', Pose, queue_size=1)
    # pub3 = rospy.Publisher('uav3_target_location', Pose, queue_size=1)


    pub.publish(NavigationTasks)
    print "sent : "
    print NavigationTasks
    # pub1.publish(str(hello))
    # pub2.publish(str(hello))
    # pub3.publish(str(hello))


def listener():


    rospy.Subscriber("kuri_msgs/ObjectsMap", ObjectsMap, callback)    #I add the kuri-message type here

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    rospy.init_node('listener', anonymous=False)
    listener()


