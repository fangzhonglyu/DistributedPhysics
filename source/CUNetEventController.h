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

class NetEventController {
public:
    enum Status {
        /** No connection requested */
        IDLE,
        /** Connecting to server */
        CONNECTING,
        /** Connected to server */
        CONNECTED,
        /** Game is in progress */
        INGAME,
        /** Error in connection */
        NETERROR
    };

protected:
    std::unordered_map<std::type_index, Uint8> _eventTypeMap;
    std::vector<std::shared_ptr<NetEvent>> _eventTypeVector;

    /** The asset manager for the controller. */
    std::shared_ptr<cugl::AssetManager> _assets;

    /** The network configuration */
    cugl::net::NetcodeConfig _config;

    /** The network connection */
    std::shared_ptr<cugl::net::NetcodeConnection> _network;

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

    const std::vector<std::byte>& wrap(std::shared_ptr<NetEvent>& e);
    
    void processReceivedData();
    
    void sendQueuedOutData();
    
    Uint64 getGameTick() {
        _appRef->getUpdateCount() - _startGameTimeStamp;
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
    std::priority_queue<std::shared_ptr<NetEvent>, std::vector<std::shared_ptr<NetEvent>>, NetEventCompare> _outEventQueue;

public:
    NetEventController(void) { };
    
    bool init(const std::shared_ptr<cugl::AssetManager>& assets);

    static std::shared_ptr<NetEventController> alloc(const std::shared_ptr<cugl::AssetManager>& assets) {
		std::shared_ptr<NetEventController> result = std::make_shared<NetEventController>();
		return (result->init(assets) ? result : nullptr);
	}

    bool connectAsHost();

    bool connectAsClient(std::string roomID);

    void disconnect();

    std::string getRoomID() const {
        return _roomid;
    }

    bool getIsHost() const {
        return _isHost;
    }

    Status getStatus() const {
        return _status;
    }
    
    void startGame();

    void updateNet();

    //template that must be of type NetEvent
    template <typename T>
    void attachEventType() {
        if (!_eventTypeMap.count(std::type_index(typeid(T)))) {
            _eventTypeVector.push_back(std::make_shared<T>());
            _eventTypeMap.insert(std::make_pair(std::type_index(typeid(T)), _eventTypeVector.size() - 1));
        }
    }

    bool isInAvailable();

    std::shared_ptr<NetEvent>& popInEvent();

    void pushOutEvent(std::shared_ptr<NetEvent>& e);
};

#endif /* __CU_NET_EVENT_CONTROLLER_H__ */
