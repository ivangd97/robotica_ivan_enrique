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
			differentialrobot_proxy->setSpeedBase(5, -rot * 2);
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
			differentialrobot_proxy->setSpeedBase(200, 0);
			currentState = State::ORIENTAR;
			target.activo = true;
			break;
		case State::AVANZAR_FRONT:
			std::cout << "AVANZAR_FRONT" << std::endl;
			differentialrobot_proxy->setSpeedBase(5, ldata.front().angle);
			differentialrobot_proxy->setSpeedBase(500, 0);
			break;
		//comenzamos estados IDLE,ORIENTAR_BEGIN,ORIENTAR para esperar click
		//case IDLE
		case State::IDLE:
			std::cout << "IDLE" << std::endl;
			//levamos al robot a ORIENTAR
			if (target.activo == true)
			{
				std::cout << "target activo" << std::endl;
				currentState = State::ORIENTAR;
				pos_robot = bState;
				target.activo = false;
			}
			break;

		//case ORIENTAR
		case State::ORIENTAR:
			std::cout << "ORIENTAR" << std::endl;
			gotoTarget(ldata);
			break;

		case State::PARAR:
			std::cout << "PARAR" << std::endl;
			distInicio = 0;
			if (target.activo == true)
			{
				currentState = State::IDLE;
			}
			differentialrobot_proxy->setSpeedBase(0, 0);
			break;
		case State::BUG:
			std::cout << "BUG" << std::endl;
			bichote(ldata);
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
	A = tr.x() - bState.x;
	B = -(tr.z() - bState.z);
	C = -(B * bState.x) - (A * bState.z);
	alfa = atan2(tr.x(), tr.z());
	if (distInicio == 0)
	{
		distInicio = sqrt(pow(A, 2.0) + pow(-B, 2.0));
		std::cout << "Distancia inicial al punto: " << distInicio << std::endl;
	}
	//float distAUX = sqrt(pow(A, 2.0) + pow(-B, 2.0));
<<<<<<< HEAD
=======
	if (((A < 100) && (A > -100)) && ((B < 110) && (B > -100)))
	{
		std::cout << "entrando en parar" << std::endl;
		currentState = State::PARAR;
		bandera = false;
		target.activo = false;
		return;
	}
>>>>>>> 4e408707ae9119225a19d42809b6a9dabdffcc14
	if (bandera == false)
	{
		if (ldata.front().dist < threshold || ldata[sizeof(ldata) / 2].dist < threshold || ldata.back().dist < threshold)
		{
			std::cout << "entrando en OBSTACLE desde bandera false" << std::endl;
			differentialrobot_proxy->setSpeedBase(0, 2 * rot);
			obstacle(tr);
			return;
		}
		//no avanza pero gira la cantidad alfa
		if (!visto && fabs(alfa) > 0.05)
		{
			differentialrobot_proxy->setSpeedBase(0, alfa);
		}
		if (fabs(alfa) < 0.05)
		{
<<<<<<< HEAD
			differentialrobot_proxy->setSpeedBase(200, 0);
			return;
=======

			if (ldata.front().dist < 600 || ldata.front().dist > 100)
			{
				differentialrobot_proxy->setSpeedBase(200, 0);
			}
			else
			{
				differentialrobot_proxy->setSpeedBase(200, 0);
			}
>>>>>>> 4e408707ae9119225a19d42809b6a9dabdffcc14
		}
	}

	else
	{
<<<<<<< HEAD
		if (!visto)
		{
			differentialrobot_proxy->setSpeedBase(0, -1.5);
		}

		if (ldata.front().dist < threshold+50 || ldata[sizeof(ldata) / 2].dist < threshold || ldata.back().dist < threshold+50)
		{
			std::cout << "entrando en OBSTACLE desde bandera true" << std::endl;
			differentialrobot_proxy->setSpeedBase(0, 1.5 * rot);
			obstacle(tr);
			return;
		}

		if (((A < 100) && (A > -100)) && ((B < 100) && (B > -100)))
		{
			std::cout << "entrando en parar" << std::endl;
			currentState = State::PARAR;
			bandera = false;
			target.activo = false;
			return;
		}

		if (fabs(alfa) < 0.05)
=======

		if (ldata.front().dist < threshold || ldata[sizeof(ldata) / 2].dist < threshold || ldata.back().dist < threshold)
		{
			//Posible cambio de rotacion
			differentialrobot_proxy->setSpeedBase(0, 2 * rot);
			obstacle(tr);
			return;
		}
		//no avanza pero gira la cantidad alfa
		if (!visto)
		{
			differentialrobot_proxy->setSpeedBase(0, -1.5);
		}
		if (fabs(alfa) < 0.05)
		{

			if (ldata.front().dist < 600 || ldata.front().dist > 100)
			{
				differentialrobot_proxy->setSpeedBase(200, 0);
			}
			else
			{
				differentialrobot_proxy->setSpeedBase(200, 0);
			}
		}
		visto = targetVisible();
		if (targetVisible())
>>>>>>> 4e408707ae9119225a19d42809b6a9dabdffcc14
		{
			differentialrobot_proxy->setSpeedBase(200, 0);
			bandera = false;
<<<<<<< HEAD
			return;
		}

		if (targetVisible())
		{
			bandera = false;
		}
	}
	visto = targetVisible();
}
void SpecificWorker::bichote(const RoboCompLaser::TLaserData &ldata)
{
	std::cout << "BICHOTE" << std::endl;
	differentialrobot_proxy->setSpeedBase(0, 1.5);
=======
		}
	}
}
void SpecificWorker::bichote(const RoboCompLaser::TLaserData &ldata)
{

	std::cout << "RODEAR" << std::endl;
	differentialrobot_proxy->setSpeedBase(0, rot);
>>>>>>> 4e408707ae9119225a19d42809b6a9dabdffcc14
	if (ldata.front().dist < threshold || ldata[sizeof(ldata) / 2].dist < threshold || ldata.back().dist < threshold)
	{
		std::cout << "RODEAR" << std::endl;
		currentState = State::BUG;
		return;
	}
	currentState = State::AVANZAR;
}

void SpecificWorker::obstacle(QVec tr)
{
	std::cout << "entrando en OBSTACLE" << std::endl;
	bandera = true;
	if (!targetVisible())
	{
		currentState = State::BUG;
	}
	else
	{
		std::cout << "saliendo de OBSTACLE" << std::endl;
		differentialrobot_proxy->setSpeedBase(0, 0);
		currentState = State::AVANZAR;
	}
	return;
}

bool SpecificWorker::targetVisible()
{
	//std::cout << "entrando en target visible" << std::endl;
	QPolygonF polygon;
	auto laser = innerModel->getNode<InnerModelLaser>(QString("laser"));
	for (auto l : ldata)
	{
		QVec lr = laser->laserTo(QString("world"), l.dist, l.angle);
		polygon << QPointF(lr.x(), lr.z());
	}
	QVec tr = QVec::vec3(target.x, 0, target.z);
	//std::cout << "X : " << target.x << " Y : " << target.z << std::endl;
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
//Implementacion de metodos entrega 3
void SpecificWorker::GotoPoint_go(string nodo, float x, float y, float alpha)
{
	target.write(x, y);
	gotoTarget(ldata);
}

void SpecificWorker::GotoPoint_turn(float speed)
{
	//implementCODE
	differentialrobot_proxy->setSpeedBase(speed, rot);
}

bool SpecificWorker::GotoPoint_atTarget()
{
	//implementCODE
	return false;
}

void SpecificWorker::GotoPoint_stop()
{
	//implementCODE
	differentialrobot_proxy->setSpeedBase(0, 0);
}