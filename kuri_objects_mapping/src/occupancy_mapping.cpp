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
}


void Object_mapping::mapcallback(const kuri_msgs::Objects objects){
    int objectsNum= objects.objects.size();
    std::thread update1(&Object_mapping::UpdateMap,this,objects,objectsNum,0); // last elemet ,0, is for adding object to map
    update1.join();
    publishMap();
    map_pub1.publish(flag);
    flag_success=true;
}


void Object_mapping::ObjectsRemovalcallback(const kuri_msgs::Objects objectsR){
    int RemovedobjectsNum= objectsR.objects.size();
    std::thread update2(&Object_mapping::UpdateMap,this,objectsR,RemovedobjectsNum,1); // last elemet ,1, is for removing object from map
    update2.join();
    publishMap();
}

void Object_mapping::UpdateMap(const kuri_msgs::Objects objects,int objectsNum , int Add_Remove)

{
  // ------------------------------------------------------------------
  // ---<  add or remove objects based on Add_Remove flag value >------
  // ------------------------------------------------------------------

    std::lock_guard<std::mutex> lock(m); // lock the thread to avoid adding and removing objects simultaneoulsy
    Position position;
    Index index;
    std::cout << position.size() << std::endl;

    ros::Time time = ros::Time::now();
    for (int i=0;i<objectsNum;i++){
    position [0] = objects.objects[i].pose.pose.position.x;
    position [1] = objects.objects[i].pose.pose.position.y;
    newObjects.objects.push_back(objects.objects[i]);
    map.getIndex(position,index);
    std::cout << objectsNum << std::endl;
    std::cout << position << std::endl;
    std::cout << index.transpose() << std::endl;
            if (Add_Remove ==0)  map.at("static", index) = 1;
            else if (Add_Remove ==1) map.at("static", index) = 0;

        }
    map.setTimestamp(time.toNSec());
    


}

void Object_mapping::publishMap (void )
{
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
        {
            GridMapRosConverter::toMessage(map,result.objects_map.map);
            for(int j=0; j<newObjects.objects.size();j++)
                result.objects_map.objects.push_back(newObjects.objects[j]);

            newObjects.objects.erase(newObjects.objects.begin(), newObjects.objects.end());
            flag_success=false;
            publishMap();
            actionServer->setSucceeded(result); //it should run continously
        }
//        publishMap();
        if (actionServer->isPreemptRequested()) {
            ROS_INFO("%s: Preempted", actionName.c_str());
            actionServer->setPreempted();
            break;
        }

        ros::spinOnce();
        loopRate.sleep();
    }
}

