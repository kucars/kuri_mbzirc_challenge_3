#include <occupancy_mapping.h>


Object_mapping::Object_mapping(void){

  map_pub = ph.advertise<nav_msgs::OccupancyGrid>("/map_occupancy", 100);
  map_pub1 = ph.advertise<std_msgs::Bool>("/Map_to_TaskAllocator_flag", 100);
  map_sub = ph.subscribe("/TrackingAction/feedback", 100, &Object_mapping::mapcallback, this);
  map_sub1 = ph.subscribe("/RemovedObjects", 100, &Object_mapping::ObjectsRemovalcallback, this);
  map.header.frame_id="/world";
  map.info.resolution= 1.0;         
  map.info.width= 60;          
  map.info.height= 90;           
  map.info.origin.position.x = -(60)/2;           
  map.info.origin.position.y = -(90)/2;   
  std::vector<int8_t> update_map(map.info.width*map.info.height,-1);
  map.data=update_map;
  flag.data=true;
}


void Object_mapping::mapcallback(const kuri_msgs::Objects objects){
	int objectsNum= objects.objects.size();
        std::thread update1(&Object_mapping::UpdateMap,this,objects,objectsNum,0); // last elemet ,0, is for adding object to map
	update1.join();  
	map_pub.publish(map);
	map_pub1.publish(flag);
  	}


void Object_mapping::ObjectsRemovalcallback(const kuri_msgs::Objects objectsR){
	int RemovedobjectsNum= objectsR.objects.size(); 
        std::thread update2(&Object_mapping::UpdateMap,this,objectsR,RemovedobjectsNum,1); // last elemet ,1, is for removing object from map
	update2.join();
	map_pub.publish(map); 
	}   

void Object_mapping::UpdateMap(const kuri_msgs::Objects objects,int objectsNum , int Add_Remove)

{
/////////////// add or remove objects based on Add_Remove flag value///////////////

        std::lock_guard<std::mutex> lock(m); // lock the thread to avoid adding and removing objects simultaneoulsy

        std::vector<int8_t> update_map = map.data;
	for (int i=0;i<objectsNum;i++){
	if (objects.objects[i].pose.pose.position.x <= 0 && objects.objects[i].pose.pose.position.y <=0)  vertex= ((-29+int(objects.objects[i].pose.pose.position.y)-1)*-1)+(44+int(objects.objects[i].pose.pose.position.x))*60;
	if (objects.objects[i].pose.pose.position.x <= 0 && objects.objects[i].pose.pose.position.y >0)   vertex= ((-29+int(objects.objects[i].pose.pose.position.y))*-1)+(44+int(objects.objects[i].pose.pose.position.x))*60;
	if (objects.objects[i].pose.pose.position.x > 0 && objects.objects[i].pose.pose.position.y <=0)   vertex= ((-29+int(objects.objects[i].pose.pose.position.y)-1)*-1)+(44+int(objects.objects[i].pose.pose.position.x)+1)*60;
	if (objects.objects[i].pose.pose.position.x > 0 && objects.objects[i].pose.pose.position.y >0)    vertex= ((-29+int(objects.objects[i].pose.pose.position.y))*-1)+(44+int(objects.objects[i].pose.pose.position.x)+1)*60;
	if (Add_Remove ==0) update_map[vertex]=100;
	else if (Add_Remove ==1) update_map[vertex]=0;
	}
	map.data = update_map;
	map.header.stamp=ros::Time::now();

}


void Object_mapping::StoreMap(actionlib::SimpleActionServer<kuri_msgs::MappingAction> *actionServer,kuri_msgs::MappingResult result){
        ros::Rate loopRate(30);
	while (ros::ok()){
	    result.objects_map.map=map;

          if (actionServer->isPreemptRequested()) {
            ROS_INFO("%s: Preempted", actionName.c_str());
            actionServer->setPreempted();
		break;
		}

	ros::spinOnce();
	loopRate.sleep();
}
}

