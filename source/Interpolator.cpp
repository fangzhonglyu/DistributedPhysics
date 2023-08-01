//
//  Interpolator.cpp
//  Rocket
//
//  Created by Barry Lyu on 6/28/23.
//

#include "Interpolator.h"

#define ITPR_STATS 0

#define ITPR_METHOD 3

using namespace cugl;

void Interpolator::addObject(std::shared_ptr<physics2::Obstacle> obj, std::shared_ptr<targetParam> param){
//  if(_cache.count(obj)){
//            auto oldParam = _cache.at(obj);
//            obj->setPosition(oldParam->P3);
//            obj->setLinearVelocity(oldParam->targetVel);
//            obj->setAngle(oldParam->targetAngle);
//            obj->setAngularVelocity(oldParam->targetAngV);
//        }
    if(_cache.count(obj)){
        auto oldParam = _cache.at(obj);
        param->I = oldParam->I;
        param->numI = oldParam->numI;
    }
    _cache.erase(obj);
    _cache.insert(std::make_pair(obj,param));
    _stepSum += param->numSteps;
    _itprCount ++;
//    Vec2 P0 = obj->getPosition();
//    Vec2 P1 = obj->getPosition()+obj->getLinearVelocity();
//    Vec3 P2 = Vec2(param->ste[0],param->second[1])-Vec2(param->second[2],param->second[3]);
//    Vec3 P3 = Vec2(param->second[0],param->second[1]);
}

bool Interpolator::contains(std::shared_ptr<physics2::Obstacle> obj){
    return _cache.count(obj) > 0;
}

void Interpolator::fixedUpdate(){
    for(auto it = _cache.begin(); it != _cache.end(); it++){
        auto obj = it->first;
        std::shared_ptr<targetParam> param = it->second;
        int stepsLeft = param->numSteps-param->curStep;
        
        if(stepsLeft <= 1){
            obj->setPosition(param->P3);
            obj->setLinearVelocity(param->targetVel);
            obj->setAngle(param->targetAngle);
            obj->setAngularVelocity(param->targetAngV);
            _deleteCache.push_back(it->first);
            _ovrdCount++;
        }
        else{
            float t = ((float)param->curStep)/param->numSteps;
            CUAssert(t<=1.f && t>=0.f);
            
            #if ITPR_METHOD == 1
            Vec2 P1 = obj->getPosition()+obj->getLinearVelocity()/1.f;
            Vec2 pos = (1-t)*(1-t)*(1-t)*obj->getPosition() + 3*(1-t)*(1-t)*t*P1 + 3*(1-t)*t*t*param->P2 + t*t*t*param->P3;
            obj->setPosition(pos);
            
            #elif ITPR_METHOD == 2
            Vec2 pos = (2*t*t*t-3*t*t+1)*obj->getPosition() + (t*t*t-2*t*t+t)*obj->getLinearVelocity() + (-2*t*t*t+3*t*t)*param->P3 + (t*t*t-t*t)*param->targetVel;
            obj->setPosition(pos);
            
            #elif ITPR_METHOD == 3
            //Vec2 vel = obj->getLinearVelocity();
            //ojb->setX(interpolate(stepsLeft,param->P3.x,obj->getX()));
            //obj->setY(interpolate(stepsLeft,param->P3.y,obj->getY()));
            
            Vec2 E = param->P3-obj->getPosition();
            
            param->numI++;
            param->I = param->I + E;
            
            Vec2 P = E*10.f;
            
            Vec2 I = param->I*0.01f;
            
            Vec2 D = obj->getLinearVelocity()*0.5f;
            
            obj->setLinearVelocity(obj->getLinearVelocity()+P-D+I);
            
            #else
            obj->setX(interpolate(stepsLeft,param->P3.x,obj->getX()));
            obj->setY(interpolate(stepsLeft,param->P3.y,obj->getY()));
            
            #endif
            
            //obj->setVX(interpolate(stepsLeft, param->targetVel.x, obj->getVX()));
            //obj->setVY(interpolate(stepsLeft, param->targetVel.y, obj->getVY()));
            obj->setAngle(interpolate(stepsLeft, param->targetAngle, obj->getAngle()));
            obj->setAngularVelocity(interpolate(stepsLeft, param->targetAngV, obj->getAngularVelocity()));
        }
        param->curStep++;
    }

    for(auto it = _deleteCache.begin(); it != _deleteCache.end(); it++){
        _cache.erase(*it);
    }
    _deleteCache.clear();

    if(ITPR_STATS){
        CULog("%ld/%ld overriden", _itprCount-_ovrdCount,_itprCount);
        CULog("Average step: %f", ((float)_stepSum)/_itprCount);
    }
}

float Interpolator::interpolate(int stepsLeft, float target, float source){
    return (target-source)/stepsLeft+source;
}
