//
//  Interpolator.h
//  Rocket
//
//  Created by Barry Lyu on 6/28/23.
//

#ifndef Interpolator_h
#define Interpolator_h

#include<cugl/cugl.h>
#include"CUNetEvent.h"

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
    Vec2 I;
    Uint64 numI;
} targetParam;

class CUObstacleFactory {
public:
    static std::shared_ptr<CUObstacleFactory> alloc() {
        return std::make_shared<CUObstacleFactory>();
    };
    /**
     * Serialize any paramater that the event contains to a vector of bytes.
     */
    virtual std::shared_ptr<physics2::Obstacle> createObstacle();
};

class NetPhysicsController {
    
protected:
    long _itprCount;
    
    long _ovrdCount;
    
    long _stepSum;

    Uint32 _objRotation;

    std::shared_ptr<cugl::physics2::ObstacleWorld> _world;
    
    std::map<std::shared_ptr<physics2::Obstacle>,std::shared_ptr<targetParam>> _cache;
    
    std::vector<std::shared_ptr<physics2::Obstacle>> _deleteCache;
    
public:
    NetPhysicsController():
        _itprCount(0),_ovrdCount(0),_stepSum(0),_objRotation(0) {};

    void init(std::shared_ptr<cugl::physics2::ObstacleWorld> world) {
        _world = world;
    }

    void reset(){
        _itprCount = 0;
        _ovrdCount = 0;
        _stepSum = 0;
        _cache.clear();
        _deleteCache.clear();
    }
    
    bool contains(std::shared_ptr<physics2::Obstacle> obj);

    void processPhysSyncEvent(const std::shared_ptr<PhysSyncEvent>& event);

    std::shared_ptr<PhysSyncEvent> packPhysSync();
    
    void addObject(std::shared_ptr<physics2::Obstacle> obj, std::shared_ptr<targetParam> param);
    
    void fixedUpdate();
    
    float interpolate(int stepsLeft, float target, float source);
};

#endif /* Interpolator_h */
