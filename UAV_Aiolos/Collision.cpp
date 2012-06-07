//
//  Collision.cpp
//  UAV_Aiolos
//
//  Created by Ryan Dutoit on 6/7/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#include "Collision.h"

Collision::Collision(){
    time = 0;
    x = 0;
    y = 0;
    uavs.clear();
}

Collision::Collision(double myTime, double myX, double myY) {
    time = myTime;
    x = myX;
    y = myY;
    uavs.clear();
}


double Collision::getX() {
    return x;
}

double Collision::getY() {
    return y;
}
double Collision::getTime() {
    return time;
}

void Collision::setX(double newX){
    x = newX;
}

void Collision::setY(double newY){
    y = newY;
}

void Collision::setTime(double newT){
    time = newT;
}