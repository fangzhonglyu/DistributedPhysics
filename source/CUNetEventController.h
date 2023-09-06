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
#include "CUNetEvent.h"

class NetEventController {

protected:
    std::unordered_map<std::type_index, Uint8> _eventTypeMap;
    std::vector<std::type_index> _eventTypeVector;
    std::shared_ptr<cugl::net::NetcodeConnection> _network;

    NetEvent& unwrap(const std::vector<std::byte>& data);

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

    void attachEventType(const std::type_index& eventType);

    bool isInEmpty();

    NetEvent& popInEvent();

    void pushOutEvent(const NetEvent& event);
};

#endif /* __CU_NET_EVENT_CONTROLLER_H__ */
