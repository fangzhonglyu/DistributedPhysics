//
//  Interpolator.h
//  Rocket
//
//  Created by Barry Lyu on 6/28/23.
//

#ifndef Interpolator_h
#define Interpolator_h

#include <queue>
#include <cugl/cugl.h>
#include "CUNetEvent.h"

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

class NetPhysicsController {
    
protected:
    long _itprCount;
    
    long _ovrdCount;
    
    long _stepSum;

    Uint64 _objRotation;

    std::shared_ptr<cugl::physics2::ObstacleWorld> _world;
    
    std::unordered_map<std::shared_ptr<physics2::Obstacle>,std::shared_ptr<targetParam>> _cache;
    
    std::vector<std::shared_ptr<physics2::Obstacle>> _deleteCache;

    std::vector<std::shared_ptr<ObstacleFactory>> _obstacleFacts;

    std::function<void(const std::shared_ptr<physics2::Obstacle>&, const std::shared_ptr<scene2::SceneNode>&)> _linkSceneToObsFunc;

    std::unordered_map<std::shared_ptr<physics2::Obstacle>,std::shared_ptr<scene2::SceneNode>> _sharedObsToNodeMap;

    float interpolate(int stepsLeft, float target, float source);

    std::vector<std::shared_ptr<NetEvent>> _outEvents;
    
public:
    NetPhysicsController():
        _itprCount(0),_ovrdCount(0),_stepSum(0),_objRotation(0) {};

    static std::shared_ptr<NetPhysicsController> alloc() {
		return std::make_shared<NetPhysicsController>();
	};

    void init(std::shared_ptr<cugl::physics2::ObstacleWorld>& world, Uint32 shortUID, std::function<void(const std::shared_ptr<physics2::Obstacle>&, const std::shared_ptr<scene2::SceneNode>&)> linkSceneToObsFunc) {
        _world = world;
        _world->setShortUID(shortUID);
        _linkSceneToObsFunc = linkSceneToObsFunc;
    }

    void dispose() {
    	_world = nullptr;
		_linkSceneToObsFunc = nullptr;
    }

    ~NetPhysicsController() {
		dispose();
	}

    uint32 attachFactory(std::shared_ptr<ObstacleFactory> fact) {
        _obstacleFacts.push_back(fact);
        return (uint32)_obstacleFacts.size() - 1;
    }

    void processPhysObjEvent(const std::shared_ptr<PhysObjEvent>& event);
    
    std::pair<std::shared_ptr<physics2::Obstacle>,std::shared_ptr<scene2::SceneNode>> addSharedObstacle(Uint32 factoryID, std::shared_ptr<std::vector<std::byte>> bytes);

    void reset(){
        _itprCount = 0;
        _ovrdCount = 0;
        _stepSum = 0;
        _cache.clear();
        _deleteCache.clear();
    }
    
    bool isInSync(std::shared_ptr<physics2::Obstacle> obj);

    void addSyncObject(std::shared_ptr<physics2::Obstacle> obj, std::shared_ptr<targetParam> param);

    std::shared_ptr<PhysSyncEvent> packPhysSync();
    
    void processPhysSyncEvent(const std::shared_ptr<PhysSyncEvent>& event);
    
    void fixedUpdate();

    std::vector<std::shared_ptr<NetEvent>>& getOutEvents() {
		return _outEvents;
	}
};

#endif /* Interpolator_h */
