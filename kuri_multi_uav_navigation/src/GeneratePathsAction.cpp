/***************************************************************************
 *   Copyright (C) 2006 - 2016 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                   *
 *      Ahmed AlDhanhani  <ahmed.aldhanhani@kustar.ac.ae>                                                                   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/

#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <kuri_msgs/GeneratePathsAction.h>
#include <kuri_msgs/PickObjectAction.h>
#include <kuri_msgs/DropObjectAction.h>

#include <std_msgs/Float32.h>
//#include "Navigator.h"
#include <include/Navigator.h>

using namespace SSPP;

class GeneratePathsAction
{
public:
    
  
  GeneratePathsAction(std::string name) :
    as_(nh_, name, boost::bind(&GeneratePathsAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }


  ~GeneratePathsAction(void)
  {
  }

  void executeCB(const kuri_msgs::GeneratePathsGoalConstPtr &goal)
  {
	  Navigator navigator;
	  const kuri_msgs::Tasks newtasks;
	  navigator.navigate(newtasks); 
  }

protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<kuri_msgs::GeneratePathsAction> as_;
  std::string action_name_;
  int data_count_, goal_;
  float sum_, sum_sq_;
  kuri_msgs::GeneratePathsFeedback feedback_;
  kuri_msgs::GeneratePathsResult result_;
  ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "GeneratePaths");

  GeneratePathsAction GeneratePaths(ros::this_node::getName());
  ros::spin();

  return 0;
}


