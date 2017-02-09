
#include <occupancy_mapping.h>


Object_mapping::Object_mapping(void){

  map_pub = ph.advertise<nav_msgs::OccupancyGrid>("/map_occupancy", 100);
  map_sub = ph.subscribe("/TrackingAction/feedback", 100, &Object_mapping::mapcallback, this);
  map.header.frame_id="/world";
  map.info.resolution= 1.0;         
  map.info.width= 60;          
  map.info.height= 90;           
  map.info.origin.position.x = -(60)/2;           
  map.info.origin.position.y = -(90)/2;   
  std::vector<int8_t> update_map(map.info.width*map.info.height,-1);
  map.data=update_map;
}



void Object_mapping::mapcallback(const kuri_msgs::Objects objects){


	map.header.stamp=ros::Time::now();
	std::vector<int8_t> update_map = map.data;	
	int objectsNum= objects.objects.size();   //
 
/////////////// place the detected objects location on the map /////////////// 
	int vertex;

	for (int i=0;i<objectsNum;i++){
	if (objects.objects[i].pose.pose.position.x <= 0 && objects.objects[i].pose.pose.position.y <=0)  vertex= ((-29+int(objects.objects[i].pose.pose.position.y)-1)*-1)+(44+int(objects.objects[i].pose.pose.position.x))*60;
	if (objects.objects[i].pose.pose.position.x <= 0 && objects.objects[i].pose.pose.position.y >0)   vertex= ((-29+int(objects.objects[i].pose.pose.position.y))*-1)+(44+int(objects.objects[i].pose.pose.position.x))*60;
	if (objects.objects[i].pose.pose.position.x > 0 && objects.objects[i].pose.pose.position.y <=0)   vertex= ((-29+int(objects.objects[i].pose.pose.position.y)-1)*-1)+(44+int(objects.objects[i].pose.pose.position.x)+1)*60;
	if (objects.objects[i].pose.pose.position.x > 0 && objects.objects[i].pose.pose.position.y >0)    vertex= ((-29+int(objects.objects[i].pose.pose.position.y))*-1)+(44+int(objects.objects[i].pose.pose.position.x)+1)*60;
	update_map[vertex]=100;
	}

	map.data = update_map;
	map_pub.publish(map);
  	}

void Object_mapping::coverage_percentage(actionlib::SimpleActionServer<kuri_msgs::MappingAction> *actionServer,kuri_msgs::MappingFeedback feedback,     				kuri_msgs::MappingResult result){
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
	






