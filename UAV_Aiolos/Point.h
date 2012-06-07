//
//  Point.h
//  UAV_Aiolos
//
//  Created by Ryan Dutoit on 6/7/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//
class Point {
private:
    double x;
    double y;
    
public:
    Point(double x, double y);
    Point();
    double getX();
    void setX(double x);
    double getY();
    void setY(double y);
};