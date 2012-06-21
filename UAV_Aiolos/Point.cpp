//
//  Point.cpp
//  UAV_Aiolos
//
//  Created by Ryan Dutoit on 6/7/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#include "Point.h"


Point::Point() {
    x=0;
    y=0;
}
Point::Point(double myX, double myY){
    x=myX;
    y=myY;
}
double Point::getX(){
    return x;
}
double Point::getY(){
    return y;
}
void Point::setX(double newX) {
    x=newX;
}
void Point::setY(double newY) {
    y=newY;
}
Point::~Point() {
    delete this->x;
    delete this->y;
}