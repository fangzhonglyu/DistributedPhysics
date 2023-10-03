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
    std::vector<std::function<std::shared_ptr<NetEvent>()>> _allocFuncVector;

    /** The asset manager for the controller. */
    std::shared_ptr<cugl::AssetManager> _assets;
    
    Uint8 _shortUUID;

    Uint8 _numReady;

    /** The network configuration */
    cugl::net::NetcodeConfig _config;

    /** The network connection */
    std::shared_ptr<cugl::net::NetcodeConnection> _network;

    NetPhysicsController _physController;

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

    std::priority_queue<std::shared_ptr<NetEvent>, std::vector<std::shared_ptr<NetEvent>>, NetEventCompare> _inEventQueue;
    std::priority_queue<std::shared_ptr<NetEvent>, std::vector<std::shared_ptr<NetEvent>>, NetEventCompare> _reservedInEventQueue;
    std::priority_queue<std::shared_ptr<NetEvent>, std::vector<std::shared_ptr<NetEvent>>, NetEventCompare> _outEventQueue;

public:
    NetEventController(void):
        _appRef{ nullptr },
        _status{ Status::IDLE },
        _isHost{ false },
        _startGameTimeStamp{ 0 },
        _shortUUID{ 0 },
        _numReady{ 0 },
        _roomid{ "" },
        _assets{ nullptr },
        _network{ nullptr }
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
    
    void initPhysics(std::shared_ptr<cugl::physics2::ObstacleWorld> world) {
		_physController.init(world);
	}

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
    void attachEventType(std::function<std::shared_ptr<NetEvent>()> allocFunc) {
        //CUAssertLog(std::is_base_of_v<NetEvent, T>, "Attached type is not a derived Class of NetEvent.");
        if (!_eventTypeMap.count(std::type_index(typeid(T)))) {
            _eventTypeMap.insert(std::make_pair(std::type_index(typeid(T)), _allocFuncVector.size()));
            _allocFuncVector.push_back(allocFunc);
        }
    }

    bool isInAvailable();

    std::shared_ptr<NetEvent> popInEvent();

    void pushOutEvent(const std::shared_ptr<NetEvent>& e);
};

#endif /* __CU_NET_EVENT_CONTROLLER_H__ */
