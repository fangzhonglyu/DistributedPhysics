//
//  Interpolator.h
//  Rocket
//
//  Created by Barry Lyu on 6/28/23.
//

#ifndef Interpolator_h
#define Interpolator_h

#include<cugl/cugl.h>

using namespace cugl;

class Interpolator {
    
    
protected:
    
    std::map<std::shared_ptr<physics2::Obstacle>,std::pair<int,std::vector<float>>> _cache;
    
    std::vector<std::shared_ptr<physics2::Obstacle>> _deleteCache;
    
    
public:
    void reset(){
        _cache.clear();
        _deleteCache.clear();
    }
    
    void addObject(std::shared_ptr<physics2::Obstacle> obj, std::pair<int,std::vector<float>> param);
    
    void fixedUpdate();
    
    float interpolate(int stepsLeft, float target, float source);
};

#endif /* Interpolator_h */
