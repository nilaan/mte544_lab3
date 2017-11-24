#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <stdlib.h>

ros::Publisher marker_pub;
visualization_msgs::Marker points;

int8_t *mapData;
uint32_t mapWidth = 0;
uint32_t mapHeight = 0;
double mapRes = 0;
uint8_t foundMap = 0;

uint16_t numNodes = 500;
double *nodesX;
double *nodesY;

uint16_t numEdges = 30;
uint8_t *connectedEdges;

double offsetX = 1.0;
double offsetY = 5.0;

double finalX = 0;
double finalY = 0;

double x = 0;
double y = 0; 
double yaw = 0;
double initX = NAN;
double initY = NAN;
double initYaw = NAN;


#define TAGID 0

short sgn(int x) { return x >= 0 ? 1 : -1; }

void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& indX, std::vector<int>& indY) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;
    
    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    indX.push_back(x0);
    indY.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
        if (d < 0) d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }

        //Add point to vector
        indX.push_back(x0);
        indY.push_back(y0);
    }
}

void plotPoints()
{
	geometry_msgs::Point p;
	points.header.frame_id = "/map";
	points.id = 0;	 //each curve must have a unique id or you will overwrite an old ones
	points.type = visualization_msgs::Marker::POINTS;
	points.action = visualization_msgs::Marker::ADD;
	points.ns = "points_and_lines";
	points.scale.x = 0.1;
	points.scale.y = 0.1;
	points.color.b = 1.0;
	points.color.a = 1.0;
	points.points.clear();
	for(int i = 0; i < numNodes; i++)
	{
		p.x = nodesX[i];
		p.y = nodesY[i];
		p.z = 0; //not used
		points.points.push_back(p);
	}
	marker_pub.publish(points);
}

void plotLines()
{
	geometry_msgs::Point p;
	int count = 1;
	points.header.frame_id = "/map";
	points.type = visualization_msgs::Marker::LINE_STRIP;
	points.action = visualization_msgs::Marker::ADD;
	points.ns = "points_and_lines";
	points.scale.x = 0.1;
	points.scale.y = 0.1;
	points.color.b = 1.0;
	points.color.a = 1.0;

	for(int i = 0; i < numNodes; i++)
	{
		for(int j = 0; j < numNodes; j++)
		{
			if (connectedEdges[i*numNodes+j] == 1)
			{
				points.id = count++;
				points.points.clear();
				p.x = nodesX[i];
				p.y = nodesY[i];
				p.z = 0.0;
				points.points.push_back(p);

				p.x = nodesX[j];
				p.y = nodesY[j];
				p.z = 0.0;
				points.points.push_back(p);
				marker_pub.publish(points);
			}
		}
	}

}

void generateNodes()
{
	nodesX = new double[numNodes];
	nodesY = new double[numNodes];
	for (int i = 0; i < numNodes; i++)
	{
		nodesX[i] = (rand() % mapWidth)*mapRes;
		nodesY[i] = (rand() % mapHeight)*mapRes;
	}
	ROS_INFO("Generated %d random nodes.", numNodes);
}

void removeCollisionNodes()
{
	double tempX[numNodes];
	double tempY[numNodes];
	int ind;
	int count = 0;
	for (int i = 0; i < numNodes; i++)
	{
		ind = nodesY[i]/mapRes*mapHeight+nodesX[i]/mapRes;
		if (mapData[ind] == 0)
		{
			tempX[count] = nodesX[i];
			tempY[count] = nodesY[i]; 
			count++;
		}
	}

	free(nodesX);
	free(nodesY);

	numNodes = count+2;
	nodesX = new double[numNodes];
	nodesY = new double[numNodes];

	for (int i = 0; i < count; i++)
	{
		nodesX[i] = tempX[i];
		nodesY[i] = tempY[i];
	}

	nodesX[count] = initX+offsetX;
	nodesY[count] = initY+offsetY;
	nodesX[count+1] = finalX+offsetX;
	nodesY[count+1] = finalY+offsetY;

	ROS_INFO("Number of nodes kept %d.", numNodes);
}

double getNormDist(int i, int j)
{
	return sqrt((nodesX[j]-nodesX[i])*(nodesX[j]-nodesX[i])+(nodesY[j]-nodesY[i])*(nodesY[j]-nodesY[i]));
}

void sort(double arr[], int ind[], int left, int right)
{
	int i = left;
	int j = right; 
	int tmpInd;
	double tmp;
	double pivot = arr[(left+right)/2];

	while (i <= j)
	{
		while (arr[i] < pivot)
			i++;
		while (arr[j] > pivot)
			j--;
		if (i <= j)
		{
			tmp = arr[i];
			arr[i] = arr[j];
			arr[j] = tmp;

			tmpInd = ind[i];
			ind[i] = ind[j];
			ind[j] = tmpInd;

			i++;
			j--;
		}
	}

	if (left < j)
		sort(arr, ind, left, j);
	if (i < right)
		sort(arr, ind, i, right);
}

int8_t checkCollision(int i, int j)
{
	int x0 = (int)(nodesX[i]/mapRes);
	int x1 = (int)(nodesX[j]/mapRes);
	int y0 = (int)(nodesY[i]/mapRes);
	int y1 = (int)(nodesY[j]/mapRes);
	int ind;
	long int size = 0;
	std::vector<int> indX;
	std::vector<int> indY;

//	ROS_INFO("%d %d %d %d", x0, y0, x1, y1);
	bresenham(x0, y0, x1, y1, indX, indY);

	size = indX.size();
//	ROS_INFO("%d", size);

	for (int k = 0; k < size; k++)
	{
		ind = indY[k]*mapHeight+indX[k];
		if (mapData[ind] == 100)
		{
//			ROS_INFO("Edge Collides %d %d", ind, mapData[ind]);
			return 1;
		}
//		ROS_INFO("%d %d", ind, mapData[ind]);
	}
//	ROS_INFO("Edge Did Not Collide");

	return 0;

}

void connectEdges()
{
	int i = 0;
	int j = 0;
	double dists[numNodes];
	int ind[numNodes];
	connectedEdges = new uint8_t[numNodes*numNodes];
	for (i = 0; i < numNodes; i++)
	{
		for (j = 0; j < numNodes; j++)
		{
			dists[j] = getNormDist(i, j);
			ind[j] = j;
		}
		sort(dists, ind, 0, numNodes-1);
		//ROS_INFO("%d %f %f %f %f %f", i, dists[0], dists[1], dists[2], dists[3], dists[4]);
		for (j = 1; j <= numEdges; j++)
		{
//			ROS_INFO("%d %d", i, ind[j]);
			if(!checkCollision(i, ind[j]))
			{
				connectedEdges[i*numNodes+ind[j]] = 1;
				connectedEdges[ind[j]*numNodes+i] = 1;
			}
			else
			{
				connectedEdges[i*numNodes+ind[j]] = 0;
				connectedEdges[ind[j]*numNodes+i] = 0;
			}
		}
	}
	ROS_INFO("Connected Closest Edges & Removed Collisions");
}


//Callback function for the Position topic (LIVE)
/*
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
	//This function is called when a new position message is received
	double X = msg.pose.pose.position.x; // Robot X psotition
	double Y = msg.pose.pose.position.y; // Robot Y psotition
 	double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

	std::cout << "X: " << X << ", Y: " << Y << ", Yaw: " << Yaw << std::endl ;
}
*/
//Callback function for Position topic (SIM)
void pose_callback(const gazebo_msgs::ModelStates& msg)
{
	int i;
	for(i = 0; i < msg. name.size(); i++) if(msg.name[i] == "mobile_base") break;

	x = msg.pose[i].position.x;
	y = msg.pose[i].position.y;
	yaw = tf::getYaw(msg.pose[i].orientation);

	if (std::isnan(initX) || std::isnan(initY))
	{
		initX = x;
		initY = y;
		initYaw = yaw;
		ROS_INFO("%f %f %f", initX, initY, initYaw);
	}
}

//Example of drawing a curve
void drawCurve(int k) 
{
   // Curves are drawn as a series of stright lines
   // Simply sample your curves into a series of points

   double x = 0;
   double y = 0;
   double steps = 50;

   visualization_msgs::Marker lines;
   lines.header.frame_id = "/map";
   lines.id = k; //each curve must have a unique id or you will overwrite an old ones
   lines.type = visualization_msgs::Marker::LINE_STRIP;
   lines.action = visualization_msgs::Marker::ADD;
   lines.ns = "curves";
   lines.scale.x = 0.1;
   lines.color.r = 1.0;
   lines.color.b = 0.2*k;
   lines.color.a = 1.0;

   //generate curve points
   for(int i = 0; i < steps; i++) {
       geometry_msgs::Point p;
       p.x = x;
       p.y = y;
       p.z = 0; //not used
       lines.points.push_back(p); 

       //curve model
       x = x+0.1;
       y = sin(0.1*i*k);
   }

   //publish new curve
   marker_pub.publish(lines);

}

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
	if (!foundMap)
	{
		std::vector<signed char> map_vector = msg.data;
		nav_msgs::MapMetaData info = msg.info;
		mapWidth = info.width;
		mapHeight = info.height;
		mapRes = info.resolution;
	        mapData = map_vector.data();
		int8_t* p = map_vector.data();
		mapData = new int8_t[mapWidth*mapHeight];
		for (int i = 0; i < mapWidth*mapHeight; i++)
			mapData[i] = p[i];
		ROS_INFO("Found Map of size %u x %u, %f, %lu", mapWidth, mapHeight, mapRes, map_vector.size());
		foundMap = 1;
		generateNodes();
		plotPoints();
		sleep(2);
	}
}


int main(int argc, char **argv)
{

	if (argc < 3)
	{
		ROS_INFO("Destination coordinates missing.");
		return -1;
	}
	else
	{
		finalX = atof(argv[1]);
		finalY = atof(argv[2]);
		ROS_INFO("Destination: %f %f", finalX, finalY);
	}

	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
//    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

	if (!foundMap)
		ROS_INFO("Waiting for map..");

	if (foundMap == 1 && !std::isnan(initX) && !std::isnan(initY))
	{
		removeCollisionNodes();
		plotPoints();
		connectEdges();
		foundMap++;
		plotLines();
	}


    	//Main loop code goes here:
    	vel.linear.x = 0.0; // set linear speed
    	vel.angular.z = 0.0; // set angular speed

    	velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
