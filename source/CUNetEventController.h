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

protected:
    std::unordered_map<std::type_index, Uint8> _eventTypeMap;
    std::vector<std::shared_ptr<NetEvent>> _eventTypeVector;
    std::shared_ptr<cugl::net::NetcodeConnection> _network;
    Application* _appRef;
    
    Uint64 _startGameTimeStamp;

    std::shared_ptr<NetEvent> unwrap(const std::vector<std::byte>& data,std::string source);

    const std::vector<std::byte>& wrap(std::shared_ptr<NetEvent>& e);
    
    void processReceivedData();
    
    void sendQueuedOutData();
    
    Uint64 getGameTick() {
        _appRef->getUpdateCounter() - _startGameTimeStamp;
    }

    Uint8 getType(const NetEvent& e) {
        return _eventTypeMap.at(std::type_index(typeid(e)));
    }

    bool operator()(std::shared_ptr<NetEvent> const& event1, std::shared_ptr<NetEvent> const& event2) {
        if (getType(*event1) != getType(*event2)) {
            return getType(*event1) < getType(*event1);
        }
        return event1->getEventTimeStamp() > event2->getEventTimeStamp();
    }

    std::priority_queue<std::shared_ptr<NetEvent>, std::vector<std::shared_ptr<NetEvent>>, NetEventController> _inEventQueue;
    std::priority_queue<std::shared_ptr<NetEvent>, std::vector<std::shared_ptr<NetEvent>>, NetEventController> _outEventQueue;

public:
    NetEventController(void) {};
    
    static std::shared_ptr<NetEventController> alloc(){
        std::shared_ptr<NetEventController> result = std::make_shared<NetEventController>();
        return (result->init() ? result : nullptr);
    }
    
    bool init();
    
    void startGame();

    void updateNet();

    //template that must be of type NetEvent
    template <typename T>
    void attachEventType() {
        _eventTypeVector.push_back(std::make_shared<T>());
        _eventTypeMap.insert(std::make_pair(std::type_index(typeid(T)), _eventTypeVector.size() - 1));
    }

    bool isInAvailable();

    std::shared_ptr<NetEvent>& popInEvent();

    void pushOutEvent(const NetEvent& event);
};

#endif /* __CU_NET_EVENT_CONTROLLER_H__ */
