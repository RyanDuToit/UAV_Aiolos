//
//  UAV.h
//  UAV_Aiolos
//
//  Created by Ryan Dutoit on 6/7/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//
#include <vector.h>
#include "AU_UAV_ROS/Point.h"
class UAV {
private:
    double x;
    double y;
    double xVel;
    double yVel;
    double xAcc;
    double yAcc;
    double bearing;
    
public:
    vector<Point> avoidancepoints;
    vector<Point> waypoints;
    
    UAV();
    UAV(double x, double y);
    UAV(double x, double y, double xVel, double yVel);
    UAV(double x, double y, double xVel, double yVel, double xAcc, double yAcc);
    
    
    double getX();
    void setX(double x);
    double getY();
    void setY(double x);
    double getXVel();
    void setXVel(double xVel);
    double getYVel();
    void setYVel(double yVel);
    
    double getXAcc();
    void setXAcc(double xAcc);
    double getYAcc();
    void setYAcc(double yACC);
    double getBearing();
    void setBearing(double bearing);
};