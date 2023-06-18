//
//  NetworkData.h
//  Rocket
//
//  Created by Barry Lyu on 6/17/23.
//

#ifndef __RD_NETWORK_DATA_H__
#define __RD_NETWORK_DATA_H__
#include <cugl/cugl.h>

using namespace cugl;

typedef struct {
    /** Discrete timestamp for the time of this message for network synchronization. */
    Uint64 timestamp;
    /** The message body */
    std::vector<std::byte> data;
    /**
     * Flag for distinguishing message priority.
     *
     * Even though reliable UDP guarantees order of message arrival, different players might have different latency, resulting in messages that occur at the same simulation timestep arriving out of the timestamp order. Therefore it is necessary to sort messages by priority. So that when an messages  this additional fields allows cache/delayed inputs, networkd latencies, etc.
     *
     * This field can be also be assigned to allow state synchronization messages to come in before input synchronazation.
     *
     * Order is defined by the comparator struct below.
     */
    Uint8 flag;
} netdata;

/**
 * Comparator method for netdata msgs, the messages are by default ordered by flag first and then by timestamp.
 * The flag should be used to determine message type. Game state messages should have higher priority by having a lower flag.
 *
 * Type of messages:
 *  Game state messages,
 *  Enforced state messages,
 *  Input Synchronization
 *
 */
struct compareTimestamp{
    bool operator()(netdata const& data1, netdata const&data2){
        if(data1.flag != data2.flag){
            return data1.flag >= data2.flag;
        }
        return data1.timestamp >= data2.timestamp;
    }
};

class NetDataCache {
    
    NetDataCache(){}
    
protected:
    /** Data cache for outbound messages to be broadcasted to the network. */
    static std::priority_queue<netdata,std::vector<netdata>,compareTimestamp> _outDataCache;
    /** Data cache for inbound messages received from message broadcase. */
    static std::priority_queue<netdata,std::vector<netdata>,compareTimestamp> _inDataCache;

public:
    /**
     * peek from earliest inbound message
     */
    static netdata peekIn(){
        return _inDataCache.top();
    }
    
    /**
     * pop the earliest inbound message
     */
    static netdata pollIn(){
        netdata temp = _inDataCache.top();
        _inDataCache.pop();
        return temp;
    }
    
    /**
     * push an outbound message
     */
    static void pushOut(netdata outMsg){
        _outDataCache.push(outMsg);
    }
    
    /**
     * pop an outbound message
     */
    static netdata popOut(){
        netdata temp = _outDataCache.top();
        _outDataCache.pop();
        return temp;
    }
    
    /**
     * clears all cached outbound messages
     */
    static void clearOut(){
        _outDataCache.empty();
    }
    
    /**
     * clears all cached inbound messages
     */
    static void clearIn(){
        _inDataCache.empty();
    }
    
    /**
     * purge on messages
     */
    static void resetCache(){
        clearOut();
        clearIn();
    }
};


#endif /* __RD_NETWORK_DATA_H__ */
