#include <occupancy_mapping.h>
#include <iostream>
using namespace grid_map;

Object_mapping::Object_mapping(void){

    map_pub = ph.advertise<grid_map_msgs::GridMap>("/Grid_map", 100 , true);
    map_pub1 = ph.advertise<std_msgs::Bool>("/Map_to_TaskAllocator_flag", 100);
    map_sub = ph.subscribe("/TrackingAction/NewObjects", 100, &Object_mapping::mapcallback, this);
    map_sub1 = ph.subscribe("/RemovedObjects", 100, &Object_mapping::ObjectsRemovalcallback, this);

  // ----------------------------
  // ---< Create grid map >------
  // ----------------------------

    map.add({"static"});
    map.setFrameId("map");
    map.setGeometry(Length(90, 60), 1);
    flag.data=true;
    flag_success=false;
    start=true;
}


void Object_mapping::mapcallback(const kuri_msgs::Objects objects){
    int objectsNum= objects.objects.size();
    std::thread update1(&Object_mapping::UpdateMap,this,objects,objectsNum,0); // last elemet ,0, is for adding object to map
    update1.join();
}


void Object_mapping::ObjectsRemovalcallback(const kuri_msgs::Objects objectsR){
    int RemovedobjectsNum= objectsR.objects.size();
    std::thread update2(&Object_mapping::UpdateMap,this,objectsR,RemovedobjectsNum,1); // last elemet ,1, is for removing object from map
    update2.join();
}

void Object_mapping::UpdateMap(const kuri_msgs::Objects objects,int objectsNum , int Add_Remove)

{
    std::lock_guard<std::mutex> lock(m); // lock the thread to avoid adding and removing objects simultaneously

  // ------------------------------------------------------------------
  // ---<  add or remove objects based on Add_Remove flag value >------
  // ------------------------------------------------------------------

  int Matching_element=0;  

   if ( start==true && Add_Remove==0) {  // The first recieved objects are consider as Map objects, without filtering ...
          for (int j=0; j<objectsNum;j++)  
               if ( objects.objects[j].color!="yellow" && objects.objects[j].color!="orange") newObjects.objects.push_back(objects.objects[j]);         
    start=false;
    goto MapStatic;
    }



  // ---<  Filtering Stage for adding objects >------ only unique static objects will be the new objects
  if (Add_Remove ==0) {

  for (int i=0; i<objectsNum ;i++) {
       for (int j=0; j<finalmapObjects.objects.size();j++) {

            Matching_element= Matching_element + (((abs(finalmapObjects.objects[j].pose.pose.position.x-objects.objects[i].pose.pose.position.x )< 0.5) && (abs(finalmapObjects.objects[j].pose.pose.position.y-objects.objects[i].pose.pose.position.y)< 0.5)) || (objects.objects[i].color=="yellow" ) || (objects.objects[i].color=="orange") );  // filtering tolerance for static objects is 0.5
           } 
           if ( !Matching_element)  newObjects.objects.push_back(objects.objects[i]);
           Matching_element=0;
 }          

           if (newObjects.objects.empty()) return;   

 MapStatic:

    Position position;
    Index index;
    ros::Time time = ros::Time::now();
    for (int i=0;i<newObjects.objects.size();i++){
    position [0] = newObjects.objects[i].pose.pose.position.x;
    position [1] = newObjects.objects[i].pose.pose.position.y;

        //TODO: conversion local (with respect to explorer uav home position) to local (with respect to the global ref ex Zurich or one of the corners) 

    map.getIndex(position,index);
    map.at("static", index) = 1;
    }

    map.setTimestamp(time.toNSec()); // update the map timing

    publishMapAfterAddition();   // publish the map with the added objects

}


  // ---<  For removing objects, the requested objects will be directly accepted as removed objects
  if (Add_Remove ==1) {
       for (int j=0; j<objectsNum;j++) 
           removedObjects.objects.push_back(objects.objects[j]);

    Position position;
    Index index;
    std::cout << position.size() << std::endl;


    ros::Time time = ros::Time::now();
    for (int i=0;i<removedObjects.objects.size();i++){
    position [0] = removedObjects.objects[i].pose.pose.position.x;
    position [1] = removedObjects.objects[i].pose.pose.position.y;

        //TODO: conversion local (with respect to explorer uav home position) to local (with respect to the global ref ex Zurich or one of the corners) 

    map.getIndex(position,index);
    map.at("static", index) = 0;
        }

   map.setTimestamp(time.toNSec()); // update the map timing

    publishMapAfterRemoval();  // publish the map with the removed objects

}
 
}

void Object_mapping::publishMap (void)
{

    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    map_pub.publish(message);
}



void Object_mapping::publishMapAfterAddition(void)
{

 for(int j=0; j<newObjects.objects.size();j++)
   finalmapObjects.objects.push_back(newObjects.objects[j]);

   newObjects.objects.erase(newObjects.objects.begin(), newObjects.objects.end());  // delete the current new objects to recieve the next ones

    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    map_pub.publish(message);

    map_pub1.publish(flag);
    flag_success=true;
}


void Object_mapping::publishMapAfterRemoval(void)
{

 for (int i=0; i<removedObjects.objects.size();i++) 
       for(int j=0; j<finalmapObjects.objects.size();j++)
          if ((abs(finalmapObjects.objects[j].pose.pose.position.x-removedObjects.objects[i].pose.pose.position.x )< 0.5) && (abs(finalmapObjects.objects[j].pose.pose.position.y-removedObjects.objects[i].pose.pose.position.y)< 0.5))  finalmapObjects.objects.erase(finalmapObjects.objects.begin() + j) ; // omit the removed objects from objects Map


   removedObjects.objects.erase(removedObjects.objects.begin(), removedObjects.objects.end());  // delete the current removed objects to recieve the next ones


    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    map_pub.publish(message);


}



void Object_mapping::StoreMap(actionlib::SimpleActionServer<kuri_msgs::MappingAction> *actionServer,kuri_msgs::MappingResult result){
    ros::Rate loopRate(30);
    while (ros::ok()){
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    map_pub.publish(message);
        if(flag_success)
        {   result.objects_map.objects.clear();
           
            for (int i=0; i<finalmapObjects.objects.size();i++)  result.objects_map.objects.push_back(finalmapObjects.objects[i]); 

            GridMapRosConverter::toMessage(map,result.objects_map.map);
            flag_success=false;
            publishMap();
            actionServer->setSucceeded(result); //it should run continously
        }
 
        if (actionServer->isPreemptRequested()) {
            ROS_INFO("%s: Preempted", actionName.c_str());
            actionServer->setPreempted();
            break;
        }

        ros::spinOnce();
        loopRate.sleep();
    }
}

