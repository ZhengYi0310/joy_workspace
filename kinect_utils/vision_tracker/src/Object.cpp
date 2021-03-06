/*************************************************************************
	> File Name: Object.cpp
	> Author: 
	> Mail: 
	> Created Time: Fri 07 Apr 2017 08:42:37 PM PDT
 ************************************************************************/

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <cmath>
#include <cassert>

#include <Object.h>

Object::Object()
{
    //set values for the default constructor
    setType("Object");
    setColor(Scalar(0, 0, 0));
}

Object::Object(string name)
{
    setType(name);

    if(name=="blue")
    {
       //HSV ranges for blue
        //setHSVmin(Scalar(92, 0, 0));
        //setHSVmax(Scalar(124, 256, 256));

        setHSVmin(Scalar(100, 150, 0));
        setHSVmax(Scalar(140, 255, 255));
        

        //BGR value for standard Blue:
        setColor(Scalar(255, 0, 0));
    }
    
    if(name=="green"){

		//HSV ranges for green 

		//setHSVmin(Scalar(34,50,50));
		//setHSVmax(Scalar(80,220,200));

        setHSVmin(Scalar(34, 50, 50));
        setHSVmax(Scalar(80, 220, 200));
        

		//BGR value for standard Green:
		setColor(Scalar(0,255,0));

	}

	if(name=="yellow"){

		//HSV ranges for yellow

		//setHSVmin(Scalar(20,124,123));
		//setHSVmax(Scalar(30,256,256));
        setHSVmin(Scalar(29, 86, 6));
        setHSVmax(Scalar(64, 255, 255));
        

		//BGR value for standard Yellow:
		setColor(Scalar(0,255,255));

	}

	if(name=="red")
    {

		//HSV ranges for red 

		//setHSVmin(Scalar(0,100,100));
		//setHSVmax(Scalar(19,255,255));
        setHSVmin(Scalar(160, 100, 100));
        setHSVmax(Scalar(179, 255, 255));
        

		//BGR value for standard Red:
		setColor(Scalar(0,0,255));
    }

    if(name=="white")
    {
        
        //HSV ranges for white
        setHSVmin(Scalar(0, 0, 200));
        setHSVmax(Scalar(255, 55, 255));

        //BGR value for standard white
        setColor(Scalar(255, 255, 255));

    }
}

Object::~Object()
{}

int Object::getXPos()
{
    return Object::xPos;
}

void Object::setXPos(int x)
{
    Object::xPos = x;
}

int Object::getYPos()
{
	return Object::yPos;
}

void Object::setYPos(int y)
{
	Object::yPos = y;
}

Scalar Object::getHSVmin()
{
	return Object::HSVmin;
}

Scalar Object::getHSVmax()
{
	return Object::HSVmax;
}

void Object::setHSVmin(Scalar min)
{
	Object::HSVmin = min;
}


void Object::setHSVmax(Scalar max)
{
	Object::HSVmax = max;
}


