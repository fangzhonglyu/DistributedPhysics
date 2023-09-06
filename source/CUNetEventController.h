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
#include <type_info>
#include <vector>
#include "CUNetEvent.h"

class NetEventController{
protected:
    std::unordered_map<std::type_info&,Uint8> _eventTypeMap;
    std::vector<std::type_info&> _eventTypeVector;
    
    NetEvent& unwrap(const std::vector<std::byte>& data);
    
    const std::vector<std::byte>& wrap(NetEvent& e);
    
public:
    NetEventController(void) { }
    
    bool init();
    
    void attachClass(const std::type_info& eventType);
    
    bool isInEmpty();
    
    NetEvent& popInEvent();
    
    void pushOutEvent(const NetEvent& event);

    
    
}


#endif /* __CU_NET_EVENT_CONTROLLER_H__ */
