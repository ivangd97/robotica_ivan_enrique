/*
 *    Copyright (C)2019 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	//       THE FOLLOWING IS JUST AN EXAMPLE
	//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
	//	try
	//	{
	//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
	//		std::string innermodel_path = par.value;
	//		innerModel = new InnerModel(innermodel_path);
	//	}
	//	catch(std::exception e) { qFatal("Error reading config params"); }
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);
}

void SpecificWorker::compute()
{
	const float threshold = 200; // millimeters
	float rot = 0.8;			 // rads per second
	try
	{
		RoboCompGenericBase::TBaseState bState;
		// read laser data
		RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
		//sort laser data from small to large distances using a lambda function.
		std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
		//this state detec if there is a wall
		if (target.activo == true)
		{
			currentState = 5;
		}
		else{
			currentState=-1;
		}

		switch (currentState)
		{
		case 0:
			std::cout << currentState << std::endl;
			std::cout << ldata.front().dist << std::endl;
			differentialrobot_proxy->setSpeedBase(5, rot);
			break;
		case 1:
			std::cout << currentState << std::endl;
			differentialrobot_proxy->setSpeedBase(900, 0.23);
			break;
		case 2:
			std::cout << currentState << std::endl;
			differentialrobot_proxy->setSpeedBase(5, ldata.back().angle);
			differentialrobot_proxy->setSpeedBase(950, 0);
			break;
		//case AVANZAR
		case 3:
			std::cout << currentState << std::endl;
			differentialrobot_proxy->setSpeedBase(800, 0);
			break;
		case 4:
			std::cout << currentState << std::endl;
			differentialrobot_proxy->setSpeedBase(5, ldata.front().angle);
			differentialrobot_proxy->setSpeedBase(950, 0);
			break;
		//comenzamos estados IDLE,ORIENTAR_BEGIN,ORIENTAR para esperar click
		//case IDLE
		case 5:
			std::cout << currentState << std::endl;
		    //levamos al robot a ORIENTAR_BEGIN
			currentState = 6; 
			differentialrobot_proxy->getBaseState( bState);
			pos_robot = bState;
			
			break;
		//case ORIENTAR_BEGIN
		case 6:
			std::cout << currentState << std::endl;
			{
			currentState = 7;
		    //QVec p = innermodel->transform("robot", QVec::vec3( t.x, 0, t.z), "world");
			std::tuple<float,float> pos = target.read();
			QVec tr = innerModel->transform("robot",QVec::vec3(std::get<0>(pos),0,std::get<1>(pos)),"world");
			alfa = atan2(tr.first(),tr.last());
			//no avanza pero gira la cantidad alfa
			differentialrobot_proxy->setSpeedBase(0, alfa);			
			}
			break;
		//case ORIENTAR
		case 7:
			std::cout << currentState << std::endl;
			if(fabs(alfa) < 0.05){
				differentialrobot_proxy->setSpeedBase(0, 0);			
				currentState = 3;
			}
			break;
		default:
			std::cout << currentState << std::endl;
			//differentialrobot_proxy->setSpeedBase(1000, 0);
			break;
		}
	}
	catch (const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}

	//RoboCompGenericBase::TBaseState bState;
	//differentialrobot_proxy->getBaseState( bState);
}

void SpecificWorker::RCISMousePicker_setPick(Pick myPick)
{
	//subscribesToCODE
	auto [x, y, z, name] = myPick.ice_tuple();
	target.activo = true;
	target.write(x, z);
	std::cout << "Coordenades " << x << " - " << y << " - " << z << name << std::endl;
}
