#!/usr/bin/env python



#Original Author: Abdelrahman Ali AlMahmoud
#
#
#Etisalat and British Telecom Innovation Center (EBTIC)
#
#Notes: By using this software, part of it, copying it or modifying it, you agree to give credit to EBTIC and the original author



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
            obj
        return




    #Create the messages to be published
    def reconstructKuriMsg():
        msg = SP.PoseStamped(
            header=SP.Header(
                frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),  # stamp should update
        )

        return 




    #Get the scores from the YAML file
    def getColorScores():
        # Getting the color scores from a file, I don't use this effectivly yet
        with open("weight.yaml", 'r') as stream:
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

        # The class "constructor" - It's actually an initializer
        def __init__(self, number, location_obj, color, type):
            self.number = number
            self.location_obj = location_obj
            self.color = color
            # self.size = size
            self.width = width
            self.height = height

    def make_targets(number, location_obj, color, type):
        target = Target(number, location_obj, color, type)
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



    # Compute distance between UAVs and objects
    loop2 = 0
    loopUAV = 0

    for obj in list_a:
        dist = numpy.linalg.norm(UAV1 - obj.location_obj)
        if obj.color != 'Nan':
            WeightOfObject = scores.get(obj.color)
            # print WeightOfObject
        weighteddist = dist / WeightOfObject
        # print obj.location_obj[0]
        rank1.append((weighteddist, obj.number, obj.location_obj[0], obj.location_obj[1], obj.location_obj[2],
                      'UAV1'))  # might want to construct the msg here
        rank1.sort()
        # print "UAV one ranking", rank1
        # print "               "

    for obj in list_a:
        dist = numpy.linalg.norm(UAV2 - obj.location_obj)
        if obj.color != 'Nan':
            WeightOfObject = scores.get(obj.color)
        # print WeightOfObject
        weighteddist = dist / WeightOfObject
        rank2.append((weighteddist, obj.number, 'UAV2'))
        rank2.sort()


        # print "UAV two ranking", rank2
        # print "               "

    for obj in list_a:
        dist = numpy.linalg.norm(UAV3 - obj.location_obj)
        if obj.color != 'Nan':
            WeightOfObject = scores.get(obj.color)
            # print WeightOfObject
        weighteddist = dist / WeightOfObject
        rank3.append((weighteddist, obj.number, 'UAV3'))
        rank3.sort()
        # print "UAV three ranking", rank3
        # print "                 "


        # print "Goal ranking   "
        # print "               "

    FinalRanking = []

    FinalRanking = rank1 + rank2 + rank3
    FinalRanking.sort()

    # Removes all the other entries except for the closes object/UAV pair
    def clearedset2(seq):
        seen = set()
        seen_add = seen.add
        r = seq[2]
        return [x for x in seq if not (x[2] in seen or seen_add(x[2]))]



        # This removes duplicate object IDs from the list so that no 2 UAVs go to the same object

    def clearedset(seq):
        seen = set()
        seen_add = seen.add
        r = seq[1]
        return [x for x in seq if not (x[1] in seen or seen_add(x[1]))]



        # Deletes the entry which contains the object id in question from the list given

    def removeObject(list, idofentry):

        list2 = []
        for x in list:
            if x[1] == idofentry:
                print x






    # print FinalRanking
    # #this prints all of the target rankings for each UAV, could create conflict if 2 UAVs try to pick up the same target

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


    # which a different UAV got assigned
    # print ast.literal_eval(hello)

    pub = rospy.Publisher('Tasks', kuri_msgs/NavTasks, queue_size=1)

    # pub1 = rospy.Publisher('uav1_target_location', Pose, queue_size=1)
    # pub2 = rospy.Publisher('uav2_target_location', Pose, queue_size=1)
    # pub3 = rospy.Publisher('uav3_target_location', Pose, queue_size=1)


    pub.publish(str(hello))

    # pub1.publish(str(hello))
    # pub2.publish(str(hello))
    # pub3.publish(str(hello))


def listener():


    rospy.Subscriber("tester_task", ObjectsMap, callback)    #I add the kuri-message type here

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    rospy.init_node('listener', anonymous=False)
    listener()


