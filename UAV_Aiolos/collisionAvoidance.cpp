/*
 collisionAvoidance
 This is where students will be able to program in a collision avoidance algorithm.  The telemetry callback
 is already setup along with a dummy version of how the service request would work.
 */

//standard C++ headers
#include <sstream>
#include <stdlib.h>
#include <time.h>
#include <map>
#include <math.h>

//ROS headers
#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/GoToWaypoint.h"
#include "AU_UAV_ROS/RequestWaypointInfo.h"
#include "dubins.h"
#include "UAV.h"

#define rho 11.176/22.5 //TODO-check if units are correct, this is currently in meters/degree
#define TIMESTEP 1 
#define UAV_AIRSPEED 11.176
#define WEST_MOST_LONGITUDE -85.490356
#define NORTH_MOST_LATITUDE 32.606573
#define METERS_TO_LATITUDE (1.0/111200.0)
#define EARTH_RADIUS 6371000.0 //meters
#define DEGREES_TO_RADIANS (M_PI/180.0)
#define RADIANS_TO_DEGREES (180.0/M_PI)


//ROS service client for calling a service from the coordinator
ros::ServiceClient client;
ros::ServiceClient getWaypointsClient;
//using namespace AU_UAV_ROS;
//keeps count of the number of services requested
int count;

std::map<int, UAV> uavMap;
//this function is run everytime new telemetry information from any plane is recieved


double distance(double first[3],double second[3]) //[0]=latitude [1]=longitude [2] = altitude
{
	//difference in latitudes in radians
	double lat1 = first[0]*DEGREES_TO_RADIANS;
	double lat2 = second[0]*DEGREES_TO_RADIANS;
	double long1 = first[1]*DEGREES_TO_RADIANS;
	double long2 = second[1]*DEGREES_TO_RADIANS;
	
	double deltaLat = lat2 - lat1;
	double deltaLong = long2 - long1;
	
	//haversine crazy math, should probably be verified further beyond basic testing
	//calculate distance from current position to destination
	double a = pow(sin(deltaLat / 2.0), 2);
	a = a + cos(lat1)*cos(lat2)*pow(sin(deltaLong/2.0), 2);
	a = 2.0 * asin(sqrt(a));
	
	return EARTH_RADIUS * a;
}

/*give an x,y coordinate and get a Latitude/Longitude
This only works for latitudes around 32.6 degrees north*/
//TODO-make this work for any NORTH_MOST_LATITUDE
double* getLatitudeLongitude(double x, double y) {
    double* result = new double[2];
    *result = NORTH_MOST_LATITUDE-(y/EARTH_RADIUS);
    *(result+1)= (0.00001065*x)+WEST_MOST_LONGITUDE;
    return result;
}


/*takes in a latitude and longitude and gives an x,y coordinate
for that lat/long.
TODO-Verify that this function is correct
*/
double* findXYCoordinate(double longitude, double latitude) {
    double lat1 = latitude * DEGREES_TO_RADIANS;
    double lat2 = NORTH_MOST_LATITUDE*DEGREES_TO_RADIANS;
    double long1 = longitude * DEGREES_TO_RADIANS;
    double long2 = WEST_MOST_LONGITUDE*DEGREES_TO_RADIANS;
        
    double deltaLat = lat2-lat1;
    double deltaLong= long2-long1;
    
    double x = pow(sin(0 / 2.0),2);
    x = x + cos(lat2)*cos(lat2)*pow(sin(deltaLong/2.0),2);
    x = 2.0 * asin(sqrt(x));
    
    double y = pow(sin(deltaLat/2.0),2);
    y = y + cos(lat1)*cos(lat2)*pow(sin(0/2.0),2);
    y = 2.0 * asin(sqrt(y));
    
    double result[2] = {EARTH_RADIUS*x,EARTH_RADIUS*y};
    double* ptr = new double[2];
    ptr[0] = result[0];
    ptr[1] = result[1];
    return ptr;
}

/* This function will fill a UAVs waypoint vector up
 with all waypoints that need to be met, 
 returns true if no error, false if there was error
 Might not help b/c currentWaypointIndex is always zero
  */
bool fillWaypoints(UAV* myUAV, int myPlaneID) {
	int j = 0;
	double latcheck = 0;
	while (latcheck>-1) {
		Point newWaypoint = Point();
		AU_UAV_ROS::RequestWaypointInfo srv;
		srv.request.planeID=myPlaneID;
		srv.request.isAvoidanceWaypoint = false;
		srv.request.positionInQueue = j;
		if(getWaypointsClient.call(srv)) {
			newWaypoint.setY(srv.response.latitude);
			latcheck = srv.response.latitude;
			newWaypoint.setX(srv.response.longitude);
			//ROS_INFO("Plane %i, waypoint %i...x: %f  y: %f",myPlaneID,j,srv.response.latitude,srv.response.longitude);
		}
		else {
			ROS_INFO("CALL FAIL");
			return false;	
		}
		myUAV->waypoints.push_back(newWaypoint);
		j++;
	}
	//ROS_INFO("before pop %f", myUAV->waypoints.back().getX());
	myUAV->waypoints.pop_back();
	return true;
}

Point getWaypoint(UAV *myUav, int myPlaneID) {
    Point myPoint = Point();
    return myPoint;
}

/* this method returns a dubins path for a UAV that would take it to the next waypoint
 should be called whenever a path needs to be calculated */
DubinsPath setupDubins(UAV* myUAV,int myPlaneID) {
    DubinsPath* myDubinsPath = new DubinsPath;
    double q0[3];
    double q1[3];
    q0[0]=myUAV->getX();
    q0[1]=myUAV->getY();
    q0[2]=myUAV->getBearing();
    int i = 0;
    Point waypoints[2];
    //grab the xy coordinates of the next 2 waypoints
    for(i=0;i<2;i++){
        AU_UAV_ROS::RequestWaypointInfo srv;
        srv.request.planeID=myPlaneID;
        srv.request.isAvoidanceWaypoint = false;
        srv.request.positionInQueue = i;
        if(getWaypointsClient.call(srv)){
            double* xyLocation = findXYCoordinate(srv.response.longitude, srv.response.latitude);
            waypoints[i].setX(xyLocation[0]);
            waypoints[i].setY(xyLocation[1]); 
        }
    }
    //set x and y to the next waypoint as the endpoint of this dubins path
    q1[0]=waypoints[0].getX();
    q1[1]=waypoints[0].getY();
    
    //get bearing for the plane to be at when it reaches the end of path
    double deltaX = waypoints[1].getX()-waypoints[0].getX();
    double deltaY = waypoints[1].getY()-waypoints[0].getY();
    double anglebetween = atan(deltaY/deltaX)*RADIANS_TO_DEGREES;
    double bearing;
    if (deltaX>=0 && deltaY>=0) {
        bearing = anglebetween;
    }
    else if (deltaX<0 && deltaY>=0) {
        bearing = anglebetween+90;
    }
    else if (deltaX<0 && deltaY<0) {
        bearing = anglebetween+180;
    }
    else if (deltaX>=0 && deltaY<0) {
        bearing = anglebetween+270;
    }
    //set bearing
    q1[2]=bearing;
    //create path
    dubins_init(q0,q1,rho,myDubinsPath);
    return *myDubinsPath;
}

/*called whenever a telemetry update is made*/
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
	ROS_INFO("Received update #[%lld]", msg->currentWaypointIndex);
    
    //this will be replaced by students to do more than just send a string
    std::stringstream ss;
    ss << "Sending service request " << count;
    
    UAV* currentUAV;
    double* currentXY = findXYCoordinate(msg->currentLongitude, msg->currentLatitude);
    
    //see if our uav is in map
    if (uavMap.count(msg->planeID)==0) {
        currentUAV = new UAV;
        uavMap[msg->planeID] = *currentUAV;
        //TODO-set x and y velocities if on first runthrough
        //TODO-use meters not long lat
        currentUAV->setXVel(0);
        currentUAV->setYVel(UAV_AIRSPEED);
        //ROS_INFO("front %f", currentUAV->waypoints.front().getX());
    }
    else {
        //set currentuav pointer and the uav's bearing
        currentUAV = &(uavMap[msg->planeID]);
        double* currentXY = findXYCoordinate(msg->currentLongitude,msg->currentLatitude);
        double deltaX = currentUAV->getX()-currentXY[0];
        double deltaY = currentUAV->getY()-currentXY[1];
        double anglebetween = atan(deltaY/deltaX)*RADIANS_TO_DEGREES;
        double bearing;
        if (deltaX>=0 && deltaY>=0) {
            bearing = anglebetween;
        }
        else if (deltaX<0 && deltaY>=0) {
            bearing = anglebetween+90;
        }
        else if (deltaX<0 && deltaY<0) {
            bearing = anglebetween+180;
        }
        else if (deltaX>=0 && deltaY<0) {
            bearing = anglebetween+270;
        }
        currentUAV->setBearing(bearing);
    }
    //if no waypoints yet, fill it up for our uav
    if(currentUAV->waypoints.size()<1) {
        fillWaypoints(currentUAV,msg->planeID);    
    }
    
    currentUAV->setX(currentXY[0]);
    currentUAV->setY(currentXY[1]);
    ROS_INFO("rocking plane %i %f %f %i", msg->planeID,    currentUAV->getX(), currentUAV->getXVel(), msg->currentWaypointIndex);
    
    
    
    //if no path exists, and we have a place to go, create the path
    if (currentUAV->avoidancepoints.size()==0 && 
        currentUAV->waypoints.size()>1) {
        DubinsPath newDubinspath = setupDubins(currentUAV, msg->planeID);
        int i=0;
        double q[3];
        int step = 0;
        while (i==0) {
            i = dubins_path_sample(&newDubinspath,UAV_AIRSPEED*step,q);
            Point newAvoidancePoint = Point(q[0],q[1]);
            currentUAV->avoidancepoints.push_back(newAvoidancePoint);
            step++;
        }
    } 
    
    //diagnostics     
    if(msg->planeID==0){
        for(int i = 0;i<currentUAV->avoidancepoints.size();i++)
        {
            ROS_INFO("dubins x: %f y: %f",currentUAV->avoidancepoints.at(i).getX(), currentUAV->avoidancepoints.at(i).getY());
            double* result = getLatitudeLongitude(currentUAV->avoidancepoints.at(i).getX(),currentUAV->avoidancepoints.at(i).getY());
            ROS_INFO("dubins locations: %f %f",result[0],result[1]);
        }
    }
    
    
    
    //if we have a place to go, then lets go there
    
    if(currentUAV->avoidancepoints.size() > 0) {
        AU_UAV_ROS::GoToWaypoint srv;
        srv.request.planeID = msg->planeID;
        double* resultlatlong = getLatitudeLongitude(currentUAV->avoidancepoints.front().getX(),currentUAV->avoidancepoints.front().getY());
        srv.request.latitude = resultlatlong[0];
        srv.request.longitude = resultlatlong[1];
        srv.request.altitude = 400;
                
                
        currentUAV->avoidancepoints.erase(currentUAV->avoidancepoints.begin());

        //these settings mean it is an avoidance maneuver waypoint AND to clear the avoidance queue
        srv.request.isAvoidanceManeuver = true;
        srv.request.isNewQueue = true;
    
    
        //check to make sure the client call worked (regardless of return values from service)
        
        if(client.call(srv)) {
            ROS_INFO("Received response from service request %d", (count++));
        }
        else {
            ROS_ERROR("Did not receive response");
        }
    }
    
    
}


int main(int argc, char **argv) {
	//standard ROS startup
	ros::init(argc, argv, "collisionAvoidance");
	ros::NodeHandle n;
	//subscribe to telemetry outputs and create client for the avoid collision service
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	client = n.serviceClient<AU_UAV_ROS::GoToWaypoint>("go_to_waypoint");
    getWaypointsClient = n.serviceClient<AU_UAV_ROS::RequestWaypointInfo>("request_waypoint_info");
    
	//random seed for if statement in telemetryCallback, remove when collision avoidance work begins
	srand(time(NULL));
	//initialize counting
	count = 0;
    ROS_INFO("startingLongitude %f", WEST_MOST_LONGITUDE);
    ROS_INFO("startingLatitude %f", NORTH_MOST_LATITUDE);
    double* result = findXYCoordinate(WEST_MOST_LONGITUDE,32.602);
    ROS_INFO("distance from top to bottom(y): %f", *(result+1));
    ROS_INFO("distance from left to right(x): %f", *result);
    
    double q1[3];
    double q2[3];
    q1[0] = NORTH_MOST_LATITUDE;
    q1[1] = WEST_MOST_LONGITUDE;
    q1[2] = 0;
    q2[0] = 32.602;
    q2[1] = WEST_MOST_LONGITUDE;
    q2[2] = 0;
    //double mydistance = distance(q1,q2);
    //ROS_INFO("distance: %f", mydistance);
    
    
    double* newresult = getLatitudeLongitude(*result, *(result+1));
    
    ROS_INFO("finalLatitude %f", *newresult);
    ROS_INFO("finalLongitude %f", *(newresult+1));
    
	//needed for ROS to wait for callbacks
	ros::spin();
	
	
		
	return 0;
}
