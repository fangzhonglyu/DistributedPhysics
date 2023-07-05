//
//  Interpolator.cpp
//  Rocket
//
//  Created by Barry Lyu on 6/28/23.
//

#include "Interpolator.h"

#define ITPR_STATS 0

using namespace cugl;

void Interpolator::addObject(std::shared_ptr<physics2::Obstacle> obj, std::shared_ptr<targetParam> param){
    if(_cache.count(obj))
        return;
    _cache.erase(obj);
    _cache.insert(std::make_pair(obj,param));
    _stepSum += param->numSteps;
    _itprCount ++;
//    Vec2 P0 = obj->getPosition();
//    Vec2 P1 = obj->getPosition()+obj->getLinearVelocity();
//    Vec3 P2 = Vec2(param->ste[0],param->second[1])-Vec2(param->second[2],param->second[3]);
//    Vec3 P3 = Vec2(param->second[0],param->second[1]);
}

void Interpolator::fixedUpdate(){
    for(auto it = _cache.begin(); it != _cache.end(); it++){
        auto obj = it->first;
        std::shared_ptr<targetParam> param = it->second;
        int stepsLeft = param->numSteps-param->curStep;
        
        if(stepsLeft <= 1){
            obj->setPosition(param->P3);
            obj->setLinearVelocity(param->targetVel);
            _deleteCache.push_back(it->first);
            _ovrdCount++;
            //obj->setSensor(false);
        }
        else{
            float t = ((float)param->curStep)/param->numSteps;
            CULog("%f",t);
            CUAssert(t<=1.f && t>=0.f);
            obj->setAngle(interpolate(stepsLeft, param->targetAngle, obj->getAngle()));
            obj->setAngularVelocity(interpolate(stepsLeft, param->targetAngV, obj->getAngularVelocity()));
            obj->setVX(interpolate(stepsLeft, param->targetVel.x, obj->getVX()));
            obj->setVY(interpolate(stepsLeft, param->targetVel.y, obj->getVY()));
            Vec2 pos = (1-t)*param->P0+t*param->P3;
            //Vec2 pos = (1-t)*(1-t)*(1-t)*param->P0 + 3*(1-t)*(1-t)*t*param->P1 + 3*(1-t)*t*t*param->P2 + t*t*t*param->P3;
            //float x = interpolate(stepsLeft,param->P3.x,obj->getX());
            //CULog("itrp x: %d %f %f %f", stepsLeft, param->P3.x, obj->getX(), x);
            //obj->setX(interpolate(stepsLeft,param->P3.x,obj->getX()));
            //obj->setY(interpolate(stepsLeft,param->P3.y,obj->getY()));
            obj->setPosition(pos);
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
