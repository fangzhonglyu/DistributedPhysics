//
//  CUNetEventController.h
//  Networked Physics Library
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

    NetEvent& unwrap(const std::vector<std::byte>& data,std::string source);

    const std::vector<std::byte>& wrap(NetEvent& e);

    Uint8 getType(const NetEvent& e) {
        return _eventTypeMap.at(std::type_index(typeid(e)));
    }

    bool operator()(NetEvent const& data1, NetEvent const& data2) {
        if (getType(data1) != getType(data2)) {
            return getType(data1) < getType(data2);
        }
        return data1.getEventTimeStamp() > data2.getEventTimeStamp();
    }

    std::priority_queue<NetEvent, std::vector<NetEvent>, NetEventController> _inEventQueue;
    std::priority_queue<NetEvent, std::vector<NetEvent>, NetEventController> _outEventQueue;

public:
    NetEventController(void) { }

    void updateNet();

    //template that must be of type NetEvent
    template <typename T>
    void attachEventType() {
        _eventTypeVector.push_back(std::make_shared<T>());
        _eventTypeMap.insert(std::make_pair(std::type_index(typeid(T)), _eventTypeVector.size() - 1));
    }

    bool isInEmpty();

    NetEvent& popInEvent();

    void pushOutEvent(const NetEvent& event);
};

#endif /* __CU_NET_EVENT_CONTROLLER_H__ */
