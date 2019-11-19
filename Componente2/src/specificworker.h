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

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#include <mutex>
#include <thread>
#include <tuple>
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void RCISMousePicker_setPick(Pick myPick);
	struct buffer_locker{
		std::mutex in_mutex;
		void write(float x_,float z_){
			std::lock_guard<std::mutex>lock(in_mutex);
			x=x_;
			z=z_;
		}
		std::tuple<float,float> read(){
			std::lock_guard<std::mutex>lock(in_mutex);
    		return std::make_tuple(x,z);
		}
		float x,z;
		bool activo=false;
    };

public slots:
	void compute();
	void initialize(int period);
	
private:
	std::shared_ptr<InnerModel> innerModel;
    RoboCompGenericBase::TBaseState bState;
	RoboCompLaser::TLaserData ldata;
	buffer_locker target;
	RoboCompGenericBase::TBaseState pos_robot;
	float alfa;
	int cont=0;
	enum class State{CHOQUE,GIRO_ROT,AVANZAR_BACK,AVANZAR,AVANZAR_FRONT,IDLE, ORIENTAR,PARAR};
	State currentState = State::IDLE;	
	//float A,B,C;
};

#endif
