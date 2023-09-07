//
//  CUGameEvent.h
//  Networked Physics Library
//
//  This is the class header for all game events for synchronization.
//
//  Created by Barry Lyu on 9/5/23.
//

#ifndef __CU_NET_EVENT_H__
#define __CU_NET_EVENT_H__

#include <cugl/cugl.h>
#include <cstdint>
#include <vector>

using namespace cugl;

/**
 * This is the class for all communication messages between machines.
 *
 * Any information that needs to be sent through the network during gameplay should be wrapped in a NetEvent object. Custom events types can be made by inheriting this class and adding parameters as necessary.
 */
class NetEvent{
private:
    /** The time of the event from the sender. */
    Uint64 _eventTimeStamp;
    /** The time when the event was received by the recipient. */
    Uint64 _receiveTimeStamp;
    /** The ID of the sender. */
    std::string _sourceID;

    virtual void setMetaData(Uint64 eventTimeStamp, Uint64 receiveTimeStamp, const std::string sourceID) final {
        _eventTimeStamp = eventTimeStamp;
        _receiveTimeStamp = receiveTimeStamp;
        _sourceID = sourceID;
    }
    
    friend class NetEventController;
    
public:

    /**
     * Serialize any paramater that the event contains to a vector of bytes.
     */
    virtual const std::vector<std::byte>& serialize();
    /**
     * Deserialize a vector of bytes and set the corresponding parameters.
     *
     * @param data  a byte vector packed by serialize()
     *
     * This function should be the "reverse" of the serialize() function: it should be able to recreate a serialized event entirely, setting all the useful parameters of this class.
     */
    virtual void deserialize(const std::vector<std::byte>& data);
    
    Uint64 getEventTimeStamp() const { return _eventTimeStamp; }
    
    Uint64 getReceiveTimeStamp() const { return _receiveTimeStamp; }
    
    const std::string getSourceId() const { return _sourceID; }   
};


#endif /* __CU_NET_EVENT_H__ */
