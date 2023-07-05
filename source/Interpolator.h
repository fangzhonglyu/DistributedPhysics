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

typedef struct{
    int curStep;
    int numSteps;
    Vec2 P0;
    Vec2 P1;
    Vec2 P2;
    Vec2 P3;
    Vec2 targetVel;
    float targetAngle;
    float targetAngV;
} targetParam;

class Interpolator {
    
protected:
    long _itprCount;
    
    long _ovrdCount;
    
    long _stepSum;
    
    std::map<std::shared_ptr<physics2::Obstacle>,std::shared_ptr<targetParam>> _cache;
    
    std::vector<std::shared_ptr<physics2::Obstacle>> _deleteCache;
    
public:
    void reset(){
        _itprCount = 0;
        _ovrdCount = 0;
        _stepSum = 0;
        _cache.clear();
        _deleteCache.clear();
    }
    
    void addObject(std::shared_ptr<physics2::Obstacle> obj, std::shared_ptr<targetParam> param);
    
    void fixedUpdate();
    
    float interpolate(int stepsLeft, float target, float source);
};

#endif /* Interpolator_h */
