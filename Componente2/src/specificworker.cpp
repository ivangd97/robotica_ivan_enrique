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
	try
	{
		RoboCompGenericBase::TBaseState bState;
		// read laser data
		RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		//sort laser data from small to large distances using a lambda function.
		//std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
		//this state detec if there is a wall
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
			break;

		//case ORIENTAR
		case State::ORIENTAR:
			std::cout << "ORIENTAR" << std::endl;
			{
				gotoTarget(ldata);
				break;
			}
		case State::PARAR:
			std::cout << "PARAR" << std::endl;
			if (target.activo == true)
			{
				currentState = State::IDLE;
			}
			differentialrobot_proxy->setSpeedBase(0, 0);
			break;
		case State::BUG:
			std::cout << "BUG" << std::endl;
			if (targetVisible() == false)
			{
				bichote();
			}
			else
			{
				currentState = State::ORIENTAR;
			}
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
}
void SpecificWorker::gotoTarget(const RoboCompLaser::TLaserData &ldata)
{
	std::tuple<float, float> pos = target.read();
	QVec tr = innerModel->transform("base", QVec::vec3(std::get<0>(pos), 0, std::get<1>(pos)), "world");
	//float dist = tr.norm();
	float A = tr.x() - bState.x;
	float B = -(tr.z() - bState.z);
	float C = -(B * bState.x) - (A * bState.z);
	alfa = atan2(tr.x(), tr.z());

	if (ldata.front().dist < threshold)
		obstacle();

	if (((A < 100) && (A > -100)) && ((B < 100) && (B > -100)))
	{
		std::cout << "entrando en parar" << std::endl;
		currentState = State::PARAR;
		target.activo = false;
		return;
	}
	//no avanza pero gira la cantidad alfa
	differentialrobot_proxy->setSpeedBase(0, alfa);
	if (fabs(alfa) < 0.05)
	{
		differentialrobot_proxy->setSpeedBase(400, 0);
	}
}
void SpecificWorker::bichote()
{
	if (ldata.front().angle >= 1.65 && ldata.front().angle <= 1.48 && currentState == State::AVANZAR_BACK)
	{
		currentState = State::AVANZAR;
		return;
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

	if (ldata.front().dist < threshold)
	{
		currentState = State::CHOQUE;
	}
}

void SpecificWorker::obstacle()
{
	std::cout << "entrando en OBSTACLE" << std::endl;
	QVec tr;
	float dist;
	tr = innerModel->transform("base", QVec::vec3(target.x, 0, target.z), "world");
	dist = tr.norm2(); //Aquí obtenemos el tamaño del vector
    std::cout << "calculado dist: "<< dist << std::endl;
	if (dist < 100)
	{
		std::cout << "distancia menor que 100" << std::endl;
		currentState = State::IDLE;
		differentialrobot_proxy->setSpeedBase(0, 0);
		return;
	}

	if (target.activo)
	{
		std::cout << "target sigue activo" << std::endl;
		currentState = State::IDLE;
		differentialrobot_proxy->setSpeedBase(0, 0);
		return;
	}
    std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
	if (ldata.front().dist <= threshold)
	{
		std::cout << "yendo a bug" << std::endl;
		currentState = State::BUG;
		//differentialrobot_proxy->setSpeedBase(0, 0);
		return;
	}
	//differentialrobot_proxy->setSpeedBase(0, 0.5);
}

bool SpecificWorker::targetVisible()
{
	std::cout << "entrando en target visible" << std::endl;
	QPolygonF polygon;
	auto laser = innerModel->getNode<InnerModelLaser>(std::string("laser"));
	for (int i; i < 180; i++)
	{
		QVec lr = laser->laserTo(std::string("world"), ldata[i].dist, ldata[i].angle);
		polygon << QPointF(lr.x(), lr.z());
	}
	QVec tr = QVec::vec3(target.x, 0, target.z);
	std::cout << "X : " << target.x << " Y : " << target.z << std::endl;

	return polygon.containsPoint(QPointF(tr.x(), tr.z()), Qt::WindingFill);
}

void SpecificWorker::RCISMousePicker_setPick(Pick myPick)
{
	//subscribesToCODE
	auto [x, y, z, name] = myPick.ice_tuple();
	target.activo = true;
	target.write(x, z);
	std::cout << "Coordenades " << x << " - " << y << " - " << z << name << std::endl;
}
