//
//  UAV.cpp
//  UAV_Aiolos
//
//  Created by Ryan Dutoit on 6/7/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#include "UAV.h"

UAV::UAV() {
    x=0;
    y=0;
    xVel=0;
    yVel=0;
    xAcc=0;
    yAcc=0;
    routpoints.clear();
    waypoints.clear();
}

UAV::UAV(double myx, double myy){
    x=myx;
    y=myy;
    xVel=0;
    yVel=0;
    xAcc=0;
    yAcc=0;
    routpoints.clear();
    waypoints.clear();
}

UAV::UAV(double myx, double myy, double myxVel, double myyVel){
    myx=0;
    myy=0;
    myxVel=0;
    myyVel=0;
    xAcc=0;
    yAcc=0;
    routpoints.clear();
    waypoints.clear();
}
UAV::UAV(double myx, double myy, double myxVel, double myyVel, double myxAcc, double myyAcc){    
    x=myx;
    y=myy;
    xVel=myyVel;
    yVel=myxVel;
    xAcc=myxAcc;
    yAcc=myyAcc;
    routpoints.clear();
    waypoints.clear();
}


double UAV::getX() {
    return x;
}
void UAV::setX(double newx){
    x=newx;
}

double UAV::getY() {
    return y;
}
void UAV::setY(double newy){
    y=newy;
}

double UAV::getXVel(){
    return xVel;
}
void UAV::setXVel(double newxVel){
    xVel = newxVel;
}



double UAV::getYVel(){
    return yVel;
}
void UAV::setYVel(double newyVel){
    yVel = newyVel;
}


double UAV::getXAcc(){
    return xAcc;
}
void UAV::setXAcc(double newxAcc){
    xAcc = newxAcc;
}


double UAV::getYAcc(){
    return yAcc;
}
void UAV::setYAcc(double newyAcc){
    yAcc = newyAcc;
}






