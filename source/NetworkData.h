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

#define FIRE_INPUT_FLAG 1
#define STATE_SYNC_FLAG 2
#define RESET_FLAG 3
#define ROCKET_FLAG 4

typedef struct {
    /** Discrete timestamp for the time of this message for network synchronization. */
    Uint64 timestamp;
    
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
    
    std::string sourceID;
    
    /** The message body */
    std::vector<std::byte> data;
    
    Uint64 receivedBy;
    
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

class NetCache {
    
protected:
    /** Data cache for outbound messages to be broadcasted to the network. */
    std::priority_queue<netdata,std::vector<netdata>,compareTimestamp> _netCache;
    
public:
    bool isEmpty(){
        return _netCache.empty();
    }
    
    netdata peek(){
        return _netCache.top();
    }
    
    netdata pop(){
        netdata temp = _netCache.top();
        _netCache.pop();
        return temp;
    }
    
    void push(netdata data, Uint64 received){
        data.receivedBy = received;
        _netCache.push(data);
    }
    
    void clear(){
        _netCache.empty();
    }
    
    void skipToTime(Uint64 timestamp){
        while(_netCache.top().timestamp < timestamp){
            _netCache.pop();
        }
    }
};


#endif /* __RD_NETWORK_DATA_H__ */
