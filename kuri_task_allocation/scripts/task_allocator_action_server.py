#!/usr/bin/env python



#
#
import math
import actionlib
import rospy
from std_msgs.msg import String
import random
import numpy
import yaml
# from matplotlib.pyplot import figure, show
import matplotlib.pyplot as plt
import ast
import geometry_msgs
from kuri_msgs.msg import *
from geometry_msgs.msg import Pose
from mavros import setpoint as SP
import rospkg


class actionAllocator(object):
    # create messages that are used to publish feedback/result
    _feedback = kuri_msgs.msg.AllocateTasksActionFeedback()
    _result = kuri_msgs.msg.AllocateTasksResult()

    locationOfUAVs = []

    def __init__(self, name):

        #rospy.Subscriber("nav_msgs/Odometry", ObjectsMap, callbackloc)  # I add the kuri-message type here


        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, kuri_msgs.msg.AllocateTasksAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()




    def callbackloc(data2):

        global locationOfUAVs
        locationOfUAVs = data2

    def execute_cb(self, goal):
        global locationOfUAVs
        data = goal.objects_map

        def callback(data):
            # Just to make picking random colors and types easier
            WeightOfObject = 1  # Later below I will replace this with the score of the object to calculated weighted distances
            Color = ['red', 'green', 'blue','yellow', 'orange']
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
                #print listofobjects
                for obj in listofobjects:
                    drone = obj[1]
                    objfortask = obj[3]
                    droneid = ""

                    output = Task()
                    output.object = objfortask
                    output.uav_name = drone
                    output.uav_id = (int)(drone[3])## randa added this
                    output.task_type = ""

                    finaloutput.tasks.append(output)
                #print finaloutput
                return finaloutput

            def clearedset(seq):
                seen = set()
                seen_add = seen.add

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

            # Get the scores from the YAML file
            def getColorScores():
                # get an instance of RosPack with the default search paths
                rospack = rospkg.RosPack()
                # get the file path for rospy_tutorials
                packagePath = rospack.get_path('kuri_task_allocation')
                # Getting the color scores from a file, I don't use this effectivly yet
                with open(packagePath + "/config/weight.yaml", 'r') as stream:
                    try:
                        scores = yaml.load(stream)
                        # print scores
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
                timetoObject = objDist/UAVspeed

                #Use this to compute the object speed but for now I will assume the worse case scenario
                a = math.sqrt(obj.velocity.linear.x*obj.velocity.linear.x+obj.velocity.linear.y*obj.velocity.linear.y+
                              obj.velocity.linear.z*obj.velocity.linear.z)

                PossibleObjDist = objSpeed*timetoObject

                print "Object can move for ", PossibleObjDist

                PossibleObjDist = PossibleObjDist+objDist

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
            #locationOfUAVs  #use this to put the data in
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
            Locations = []
            colours = []
            area = []

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
            # print scores





            #### Important :: I can optimise this by having the distances all calculated in one loop. no need to reiterate the list
            Idontknow = 0
            for obj in data.objects:
                print Idontknow
                Idontknow +=1
                location_obj = numpy.array(
                    (obj.pose.pose.position.x, obj.pose.pose.position.y, obj.pose.pose.position.z))
                Locations.append(location_obj)

                colours.append(obj.color)

                area.append(10)


                if obj.velocity.linear.x != 0 or obj.velocity.linear.y != 0 or obj.velocity.linear.z != 0:
                    print "In dynamic object loop, dist = ", dist
                    dist = numpy.linalg.norm(UAV1 - location_obj)
                    possibleObjectLocation = getMovingObjectChance(obj, dist)
                    dist = possibleObjectLocation+dist
                    print "distance after moving = ", dist


                else:
                    dist = numpy.linalg.norm(UAV1 - location_obj)
                    print "in static loop, dist = ", dist

                WeightOfObject = scores.get(obj.color)
                # print WeightOfObject

                if obj.color != 'Nan':
                    WeightOfObject = scores.get(obj.color)


                weighteddist = dist / WeightOfObject
                print "weighted distance of the object = ", weighteddist
                rank1.append((weighteddist, 'UAV1', obj.header.seq, obj))
                rank1.sort()
                # print "UAV one ranking", rank1
                # print "               "



            for obj in data.objects:
                location_obj = numpy.array(
                    (obj.pose.pose.position.x, obj.pose.pose.position.y, obj.pose.pose.position.z))


                if obj.velocity.linear.x != 0 or obj.velocity.linear.y != 0 or obj.velocity.linear.z != 0:
                    print "In dynamic object loop, dist = ", dist
                    dist = numpy.linalg.norm(UAV2 - location_obj)
                    possibleObjectLocation = getMovingObjectChance(obj, dist)
                    dist = possibleObjectLocation+dist
                    print "distance after moving = ", dist


                else:
                    dist = numpy.linalg.norm(UAV2 - location_obj)
                    print "in static loop, dist = ", dist



                if obj.color != 'Nan':
                    WeightOfObject = scores.get(obj.color)
                weighteddist = dist / WeightOfObject
                rank2.append((weighteddist, 'UAV2', obj.header.seq, obj))
                rank2.sort()
                # print "UAV two ranking", rank2
                # print "               "



            for obj in data.objects:
                location_obj = numpy.array(
                    (obj.pose.pose.position.x, obj.pose.pose.position.y, obj.pose.pose.position.z))

                if obj.velocity.linear.x != 0 or obj.velocity.linear.y != 0 or obj.velocity.linear.z != 0:
                    print "In dynamic object loop, dist = ", dist
                    dist = numpy.linalg.norm(UAV3 - location_obj)
                    possibleObjectLocation = getMovingObjectChance(obj, dist)
                    dist = possibleObjectLocation
                    print "distance after moving = ", dist


                else:
                    dist = numpy.linalg.norm(UAV3 - location_obj)
                    print "in static loop, dist = ", dist




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
            # print FinalRanking
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

            # This block of code was removed for testing, it computes the distance from the goal
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

            #pub = rospy.Publisher('kuri_msgs/NavTasks', NavTasks, queue_size=1, latch=True)

            # pub1 = rospy.Publisher('uav1_target_location', Pose, queue_size=1)
            # pub2 = rospy.Publisher('uav2_target_location', Pose, queue_size=1)
            # pub3 = rospy.Publisher('uav3_target_location', Pose, queue_size=1)



            #x.append(UAV1[0])  # adding UAV one location to the array to be plotted
            #y.append(UAV1[1])

            Locations.append(UAV1)

            #x.append(UAV2[0])
            #y.append(UAV2[1])

            Locations.append(UAV2)

            #x.append(UAV3[0])
            #y.append(UAV3[1])

            Locations.append(UAV3)


            x = []
            y = []
            #z = []

            for obj in Locations:
                x.append(obj[0])
                y.append(obj[1])
             #   z.append(obj[2])




            #colors = [1] * 20

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
            #fig.canvas.mpl_connect('pick_event', onpick)
            UAVx = [x[-3], x[-2], x[-1]]
            UAVy = [y[-3], y[-2], y[-1]]

            #plt.plot(x[1], x[0], '-o')
            #plt.plot([x[0], x[1], x[2]], [y[0], y[1], y[2]], '-o')



            ax2 = fig.add_subplot(111)

            print NavigationTasks

            #Ultra lazy mode, fix this later..
            #Basically just graphing the UAV going to one of these points. Check which UAV has the task and plot a line
            #First task
            if NavigationTasks.tasks[0].uav_name == "UAV1":
                 xx = [UAVx[0], NavigationTasks.tasks[0].object.pose.pose.position.x]
                 yy = [UAVy[0], NavigationTasks.tasks[0].object.pose.pose.position.y]
                 ax2.plot(xx, yy, '-y')

            if NavigationTasks.tasks[0].uav_name == "UAV2":
                 xx = [UAVx[1], NavigationTasks.tasks[0].object.pose.pose.position.x]
                 yy = [UAVy[1], NavigationTasks.tasks[0].object.pose.pose.position.y]
                 ax2.plot(xx, yy, '-c')

            if NavigationTasks.tasks[0].uav_name == "UAV3":
                 xx = [UAVx[2], NavigationTasks.tasks[0].object.pose.pose.position.x]
                 yy = [UAVy[2], NavigationTasks.tasks[0].object.pose.pose.position.y]
                 ax2.plot(xx, yy, '-k')


            #Second Task
            if NavigationTasks.tasks[1].uav_name == "UAV1":
                xx = [UAVx[0], NavigationTasks.tasks[1].object.pose.pose.position.x]
                yy = [UAVy[0], NavigationTasks.tasks[1].object.pose.pose.position.y]
                ax2.plot(xx, yy, '-y')

            if NavigationTasks.tasks[1].uav_name == "UAV2":
                xx = [UAVx[1], NavigationTasks.tasks[1].object.pose.pose.position.x]
                yy = [UAVy[1], NavigationTasks.tasks[1].object.pose.pose.position.y]
                ax2.plot(xx, yy, '-c')

            if NavigationTasks.tasks[1].uav_name == "UAV3":
                xx = [UAVx[2], NavigationTasks.tasks[1].object.pose.pose.position.x]
                yy = [UAVy[2], NavigationTasks.tasks[1].object.pose.pose.position.y]
                ax2.plot(xx, yy, '-k')


            #Third task
            if NavigationTasks.tasks[2].uav_name == "UAV1":
                xx = [UAVx[0], NavigationTasks.tasks[2].object.pose.pose.position.x]
                yy = [UAVy[0], NavigationTasks.tasks[2].object.pose.pose.position.y]
                ax2.plot(xx, yy, '-y')

            if NavigationTasks.tasks[2].uav_name == "UAV2":
                xx = [UAVx[1], NavigationTasks.tasks[2].object.pose.pose.position.x]
                yy = [UAVy[1], NavigationTasks.tasks[2].object.pose.pose.position.y]
                ax2.plot(xx, yy, '-c')

            if NavigationTasks.tasks[2].uav_name == "UAV3":
                xx = [UAVx[2], NavigationTasks.tasks[2].object.pose.pose.position.x]
                yy = [UAVy[2], NavigationTasks.tasks[2].object.pose.pose.position.y]
                ax2.plot(xx, yy, '-k')


            ax1 = fig.add_subplot(111)
            col = ax1.scatter(x, y, area, colours, picker=True)

            #fig.canvas.mpl_connect('pick_event', onpick)

            fig.show()

            #pub.publish(NavigationTasks)
            print "################################# Returned ###################################"
            print NavigationTasks
            print "################################# Ended    ###################################"

            return NavigationTasks
            #print NavigationTasks
            # pub1.publish(str(hello))
            # pub2.publish(str(hello))
            # pub3.publish(str(hello))







        callbackreturns = callback(data)
        success = True
        # start executing the action

#        for i in xrange(1, goal.order):
#            # check that preempt has not been requested by the client
#            if self._as.is_preempt_requested():
#                rospy.loginfo('%s: Preempted' % self._action_name)
#                self._as.set_preempted()
#                success = False
#                break
#            self._feedback.sequence.append(callbackreturns)
#            # publish the feedback
#            self._as.publish_feedback(self._feedback)
#            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
#            r.sleep()

        if success:
            self._result.tasks = callbackreturns
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)





if __name__ == '__main__':
    rospy.init_node("actionAllocationServ")
    actionAllocator(rospy.get_name())

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


