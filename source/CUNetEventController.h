//
//  CUNetEventController.h
//  Networked Physics Library
//
//  The header for a general network controllers for multiplayer physics based game
//
//  Created by Barry Lyu on 9/5/23.
//

#ifndef __CU_NET_EVENT_CONTROLLER_H__
#define __CU_NET_EVENT_CONTROLLER_H__
#include <cugl/cugl.h>
#include <unordered_map>
#include <typeindex>
#include <vector>
#include <concepts>
#include <queue>
#include "CUNetEvent.h"
#include "Interpolator.h"

class NetEventController {
public:
    enum Status {
        /** No connection requested */
        IDLE,
        /** Connecting to server */
        CONNECTING,
        /** Connected to server */
        CONNECTED,
        /** session is started */
        INSESSION,
        /** Ready for game start */
        READY,
        /** Game is in progress */
        INGAME,
        /** Error in connection */
        NETERROR
    };

protected:
    std::unordered_map<std::type_index, Uint8> _eventTypeMap;
    std::vector<std::shared_ptr<NetEvent>> _newEventVector;

    /** The asset manager for the controller. */
    std::shared_ptr<cugl::AssetManager> _assets;
    
    Uint32 _shortUID;

    Uint8 _numReady;

    bool _physEnabled;

    /** The network configuration */
    cugl::net::NetcodeConfig _config;

    /** The network connection */
    std::shared_ptr<cugl::net::NetcodeConnection> _network;

    std::shared_ptr<NetPhysicsController> _physController;

    /** The network status */
    Status _status;
    
    /** The room id */
    std::string _roomid;

    /** Whether this device is host */
    bool _isHost;

    /** Reference to the App */
    Application* _appRef;

    /** The App fixed-time stamp when the game starts */
    Uint64 _startGameTimeStamp;

    std::shared_ptr<NetEvent> unwrap(const std::vector<std::byte>& data,std::string source);

    const std::vector<std::byte> wrap(const std::shared_ptr<NetEvent>& e);
    
    void processReceivedData();

    void processReceivedEvent(const std::shared_ptr<NetEvent>& e);

    void processGameStateEvent(const std::shared_ptr<GameStateEvent>& e);
    
    void sendQueuedOutData();
    
    Uint64 getGameTick() const {
        return _appRef->getUpdateCount() - _startGameTimeStamp;
    }

    Uint8 getType(const NetEvent& e) {
        return _eventTypeMap.at(std::type_index(typeid(e)));
    }

    struct NetEventCompare {
		bool operator()(std::shared_ptr<NetEvent> const& event1, std::shared_ptr<NetEvent> const& event2) {
			return event1->getEventTimeStamp() > event2->getEventTimeStamp();
		}
	};

    std::queue<std::shared_ptr<NetEvent>> _inEventQueue;
    std::queue<std::shared_ptr<NetEvent>> _reservedInEventQueue;
    std::queue<std::shared_ptr<NetEvent>> _outEventQueue;

public:
    NetEventController(void):
        _appRef{ nullptr },
        _status{ Status::IDLE },
        _isHost{ false },
        _startGameTimeStamp{ 0 },
        _shortUID{ 0 },
        _numReady{ 0 },
        _roomid{ "" },
        _assets{ nullptr },
        _network{ nullptr },
        _physController { nullptr },
        _physEnabled{ false }
    {};
    
    bool init(const std::shared_ptr<cugl::AssetManager>& assets);

    static std::shared_ptr<NetEventController> alloc(const std::shared_ptr<cugl::AssetManager>& assets) {
		std::shared_ptr<NetEventController> result = std::make_shared<NetEventController>();
		return (result->init(assets) ? result : nullptr);
	}

    bool connectAsHost();

    bool connectAsClient(std::string roomID);

    void disconnect();

    bool checkConnection();

    void enablePhysics(std::shared_ptr<cugl::physics2::ObstacleWorld>& world) {
        enablePhysics(world,nullptr);
    }
    
    void enablePhysics(std::shared_ptr<cugl::physics2::ObstacleWorld>& world, std::function<void(const std::shared_ptr<physics2::Obstacle>&, const std::shared_ptr<scene2::SceneNode>&)> linkSceneToObsFunc) {
        CUAssertLog(_shortUID, "You must receive a UID assigned from host before enabling physics.");
        _physEnabled = true;
        _physController->init(world,_shortUID,linkSceneToObsFunc);
        attachEventType<PhysSyncEvent>();
        attachEventType<PhysObjEvent>();
	}

    void disablePhysics() {
		_physEnabled = false;
	}

    std::shared_ptr<NetPhysicsController> getPhysController() { return _physController; }

    std::string getRoomID() const { return _roomid; }

    bool isHost() const {
        return _isHost;
    }

    int getNumPlayers() const {
        if (_network) {
            return (int)(_network->getNumPlayers());
        }
        return 1;
    }

    Status getStatus() const { return _status; }
    
    void startGame();

    void markReady();

    void updateNet();

    //template that must be of type NetEvent
    template <typename T>
    void attachEventType() {
        if (!_eventTypeMap.count(std::type_index(typeid(T)))) {
            //T t;
            //CUAssertLog(std::dynamic_cast<NetEvent>(&t), "Attached type is not a derived Class of NetEvent.");
            _eventTypeMap.insert(std::make_pair(std::type_index(typeid(T)), _newEventVector.size()));
            _newEventVector.push_back(std::make_shared<T>());
        }
    }

    bool isInAvailable();

    std::shared_ptr<NetEvent> popInEvent();

    void pushOutEvent(const std::shared_ptr<NetEvent>& e);
};

#endif /* __CU_NET_EVENT_CONTROLLER_H__ */
