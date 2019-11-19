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
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		innerModel = std::make_shared<InnerModel>(par.value);
	}
	catch (std::exception e)
	{
		qFatal("Error reading config params");
	}
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
	const float threshold = 350; // millimeters
	float rot = 0.8;			 // rads per second
	try
	{
		RoboCompGenericBase::TBaseState bState;
		// read laser data
		RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
		differentialrobot_proxy->getBaseState(bSta
		te);
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		//sort laser data from small to large distances using a lambda function.
		//std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
		//this state detec if there is a wall

		if (target.activo)
		{
			currentState = State::IDLE;
			cont++;
		}
		else
		{
			if (cont < 1)
			{
				currentState = State::PARAR;
			}
			else
			{

				if (currentState == State::ORIENTAR)
				{
					currentState = State::ORIENTAR;
				}
				else
				{
					if (ldata.front().angle >= 1.65 && ldata.front().angle <= 1.48 && currentState == State::AVANZAR_BACK)
					{
						currentState = State::AVANZAR;
					}
					//this state is there are any object close
					if (ldata.front().dist < 600)
					{
						if (currentState == State::CHOQUE)
						{
							currentState = State::AVANZAR_FRONT;
						}
						currentState = State::GIRO_ROT;
					}
					else
					{
						//in this state if the distance if bigger than 600 the robot move
						if (ldata.front().dist < 1000 && currentState != State::CHOQUE)
						{
							currentState = State::AVANZAR_BACK;
						}
						else
						{
							currentState = State::AVANZAR;
						}
					}
				}
			}

			if (ldata.front().dist < threshold)
			{
				currentState = State::CHOQUE;
			}
		}
		switch (currentState)
		{
		case State::CHOQUE:
			std::cout << "CHOQUE" << std::endl;
			std::cout << ldata.front().dist << std::endl;
			differentialrobot_proxy->setSpeedBase(5, -rot);
			break;
		case State::GIRO_ROT:
			std::cout << "GIRO_ROT" << std::endl;
			differentialrobot_proxy->setSpeedBase(800, 0.23);
			break;
		case State::AVANZAR_BACK:
			std::cout << "AVANZAR_BACK" << std::endl;
			differentialrobot_proxy->setSpeedBase(5, ldata.back().angle);
			differentialrobot_proxy->setSpeedBase(850, 0);
			break;
		//case AVANZAR
		case State::AVANZAR:
			std::cout << "AVANZAR" << std::endl;
			differentialrobot_proxy->setSpeedBase(800, 0);
			target.activo = true;
			break;
		case State::AVANZAR_FRONT:
			std::cout << "AVANZAR_FRONT" << std::endl;
			differentialrobot_proxy->setSpeedBase(5, ldata.front().angle);
			differentialrobot_proxy->setSpeedBase(850, 0);
			break;
		//comenzamos estados IDLE,ORIENTAR_BEGIN,ORIENTAR para esperar click
		//case IDLE
		case State::IDLE:
			std::cout << "IDLE" << std::endl;
			//levamos al robot a ORIENTAR
			if (target.activo == true)
			{
				currentState = State::ORIENTAR;
				pos_robot = bState;
				target.activo = false;
			}
			
		//case ORIENTAR
		case State::ORIENTAR:
			std::cout << "ORIENTAR" << std::endl;
			{
				//QVec p = innermodel->transform("robot", QVec::vec3( t.x, 0, t.z), "world");
				std::tuple<float, float> pos = target.read();
				QVec tr = innerModel->transform("base", QVec::vec3(std::get<0>(pos), 0, std::get<1>(pos)), "world");
				alfa = atan2(tr.x(), tr.z());

				if (( tr.x()-bState.x < 50) && ( tr.x()-bState.x  > -50 )&&
				( tr.z()-bState.z < 50 )&& (tr.z()-bState.z > -50))
				{
					std::cout << "entrando en parar" << std::endl;
					currentState = State::PARAR;
					cont = 0;
					break;
				}
				//no avanza pero gira la cantidad alfa
				differentialrobot_proxy->setSpeedBase(0, alfa);
				if (fabs(alfa) < 0.1)
				{
					differentialrobot_proxy->setSpeedBase(800, 0);
				}

				break;
			}
		case State::PARAR:
			std::cout << "PARAR" << std::endl;
			differentialrobot_proxy->setSpeedBase(0, 0);
			break;
		default:
			std::cout << "DEFAULT" << std::endl;
			//differentialrobot_proxy->setSpeedBase(0, 0);
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
