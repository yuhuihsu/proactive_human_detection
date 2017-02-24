#include "ros/ros.h"
#include "math.h"
#include "std_msgs/String.h"
#include "leg_tracker/Person.h"
#include "leg_tracker/PersonArray.h"
#include "tf/transform_listener.h"

static float RANGE = 0.5 * 0.5;
static float CENTER_X = 1.2;
static float CENTER_Y = 0;
static float COS_THETA = 0;
static float SIN_THETA = 0;
static tf::Vector3 transformVector;
ros::Publisher detection_pub;

void publishTopic(float x, float y, int id) {
  leg_tracker::Person person;
  person.pose.position.x = COS_THETA * x - SIN_THETA * y;
  person.pose.position.y = SIN_THETA * x + COS_THETA * y;
  person.id = id;
  detection_pub.publish(person);
}

void peopleTrackedCallback(const leg_tracker::PersonArray::ConstPtr& personArray)
{
  int size = personArray->people.size();
  bool isDetected = false;
  if (size != 0) {
    ROS_INFO("size %d", size);
    leg_tracker::PersonArray detected_People;
    detected_People.header.frame_id = personArray->header.frame_id;
    detected_People.header.stamp = personArray->header.stamp;
    for (int iNum = 0; iNum < size; iNum++) {
      float x = personArray->people[iNum].pose.position.x;
      float y = personArray->people[iNum].pose.position.y;
      int id = personArray->people[iNum].id;
      ROS_INFO("x : %f, y : %f, id : %d", x, y, id);
      float distance = pow((abs(x) - CENTER_X), 2) + pow((abs(y) - CENTER_Y), 2);
      ROS_INFO("distance : %f", distance);

      if (distance <= RANGE) {
        ROS_INFO("Detect person");
        isDetected = true;
        leg_tracker::Person person;
        person.pose.position.x = COS_THETA * x - SIN_THETA * y + transformVector.getX();
        person.pose.position.y = SIN_THETA * x + COS_THETA * y + transformVector.getY();
        person.id = id;
        detected_People.people.push_back(person);
        //publishTopic(x, y, id);
      } else {
        ROS_INFO("No person");
      }
    }
    if (isDetected){
      detection_pub.publish(detected_People);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "proactive_human_detecter");
  ros::NodeHandle n;
  tf::TransformListener listener;
  tf::StampedTransform transform;

  try {
    listener.waitForTransform("rplidar", "base_link", ros::Time(), ros::Duration(3.0));
      listener.lookupTransform("rplidar", "base_link", ros::Time(), transform);
      transformVector = transform.getOrigin();
      float x = transformVector.getX();
      float y = transformVector.getY();
      double yaw, pitch, roll;
      transform.getBasis().getRPY(roll, pitch, yaw);
      ROS_INFO("x: %f, y: %f",x , y);
      ROS_INFO("roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
      COS_THETA = cos(yaw);
      SIN_THETA = sin(yaw);

  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return 0;
  }
  ros::Subscriber sub = n.subscribe("people_tracked", 100, peopleTrackedCallback);
  detection_pub = n.advertise<leg_tracker::PersonArray>("PersonDetect", 100);
  ros::spin();
  return 0;
}
                 