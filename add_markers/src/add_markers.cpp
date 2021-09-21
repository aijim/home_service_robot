#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher marker_pub;

class CubeMarker
{
public:
    CubeMarker()
    {
        // Set our initial shape type to be a cube
	uint32_t shape = visualization_msgs::Marker::CUBE;

        marker.header.frame_id = "map";

	marker.ns = "add_markers";
	marker.id = 0;

	marker.type = shape;

	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;

	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

         marker.lifetime = ros::Duration();
    }

    void add_marker(float x, float y)
    {
        if (ros::ok())
	{
            marker.header.stamp = ros::Time::now();
	    marker.action = visualization_msgs::Marker::ADD;

	    marker.pose.position.x = x;
	    marker.pose.position.y = y;
	    marker.pose.position.z = 0;
	    marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.0;
	    marker.pose.orientation.w = 1.0;

	    // Publish the marker
	    while (marker_pub.getNumSubscribers() < 1)
	    {
	      if (!ros::ok())
	      {
		return;
	      }
	      ROS_WARN("Please create a subscriber to the marker");
	      sleep(1);
	    }
	    marker_pub.publish(marker);
	}
    }

    void delete_marker()
    {
        if (ros::ok())
	{
            marker.header.stamp = ros::Time::now();
	    marker.action = visualization_msgs::Marker::DELETE;

	    // Publish the marker
	    while (marker_pub.getNumSubscribers() < 1)
	    {
	      if (!ros::ok())
	      {
		return;
	      }
	      ROS_WARN("Please create a subscriber to the marker");
	      sleep(1);
	    }
	    marker_pub.publish(marker);
	}
    }
        
private:
    visualization_msgs::Marker marker;
};

CubeMarker cube;

void call_back(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ROS_INFO("[%1.2f, %1.2f]", msg->pose.pose.position.x,msg->pose.pose.position.y);
    if((msg->pose.pose.position.x < 1.2 && msg->pose.pose.position.x > 0.8) && 
       (msg->pose.pose.position.y < 1.2 && msg->pose.pose.position.y > 0.8))
    {
        cube.delete_marker();
    }if((msg->pose.pose.position.x > -1.2 && msg->pose.pose.position.x < -0.8) && 
       (msg->pose.pose.position.y > -1.2 && msg->pose.pose.position.y < -0.8))
    {
        cube.add_marker(-1.0, -1.0);
    }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::Subscriber acml_sub = n.subscribe("amcl_pose", 10, call_back);

    cube.add_marker(1.0, 1.0);
    ros::spin();
}
