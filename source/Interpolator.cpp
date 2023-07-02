//
//  Interpolator.cpp
//  Rocket
//
//  Created by Barry Lyu on 6/28/23.
//

#include "Interpolator.h"

using namespace cugl;

void Interpolator::addObject(std::shared_ptr<physics2::Obstacle> obj, std::pair<int,std::vector<float>> param){
    _cache.erase(obj);
    _cache.insert(std::make_pair(obj,param));
}

void Interpolator::fixedUpdate(){
    for(auto it = _cache.begin(); it != _cache.end(); it++){
        int stepsLeft = it->second.first;
        auto obj = it->first;
        std::vector<float> param = it->second.second;
        std::vector<float> newParam;
        if(stepsLeft <= 1){
            newParam = param;
            _deleteCache.push_back(it->first);
        }
        else{
            newParam = std::vector<float>();
            newParam.push_back(interpolate(stepsLeft, param[0], obj->getX()));
            newParam.push_back(interpolate(stepsLeft, param[1], obj->getY()));
            newParam.push_back(interpolate(stepsLeft, param[2], obj->getVX()));
            newParam.push_back(interpolate(stepsLeft, param[3], obj->getVY()));
            newParam.push_back(interpolate(stepsLeft, param[4], obj->getAngle()));
            newParam.push_back(interpolate(stepsLeft, param[5], obj->getAngularVelocity()));
        }
        obj->setPosition(newParam[0],newParam[1]);
        obj->setLinearVelocity(newParam[2],newParam[3]);
        obj->setAngle(newParam[4]);
        obj->setAngularVelocity(newParam[5]);
        it->second.first--;
    }
    
    for(auto it = _deleteCache.begin(); it != _deleteCache.end(); it++){
        _cache.erase(*it);
    }
    _deleteCache.clear();
}

float Interpolator::interpolate(int stepsLeft, float target, float source){
    return (target-source)/stepsLeft+source;
}
