//
//  NetworkData.h
//  Rocket
//
//  Created by Barry Lyu on 6/17/23.
//

#ifndef __RD_NETWORK_DATA_H__
#define __RD_NETWORK_DATA_H__
#include <cugl/cugl.h>
#include <cugl/base/CUEndian.h>
#include <stack>
#include <cstdint>
#include <iostream>

using namespace cugl;

#define CANNON_FLAG 101
#define STATE_SYNC_FLAG 102
#define FIRE_INPUT_FLAG 103
#define RESET_FLAG 104
#define JOINT_DESTROY_FLAG 105


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
            return data1.flag < data2.flag;
        }
        return data1.timestamp > data2.timestamp;
    }
};

class NetCache {
    
protected:
    std::stack<netdata> _netCacheHistory;
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
        _netCacheHistory.push(temp);
        return temp;
    }
    
    void push(netdata data, Uint64 received){
        _netCache.push(data);
    }
    
    void clear(){
        _netCache.empty();
    }
    
    void skipToTime(Uint64 timestamp){
        while(timestamp < _netCacheHistory.top().timestamp){
            _netCache.push(_netCacheHistory.top());
            _netCacheHistory.pop();
        }
        while(_netCache.top().timestamp < timestamp){
            _netCache.pop();
        }
    }
};

class Serializer{
private:
    std::vector<std::byte> _data;
    
public:
    Serializer() {}

    static std::shared_ptr<Serializer> alloc() {
        return std::make_shared<Serializer>();
    }
    
    void writeBool(bool b){
        _data.push_back(b ? std::byte(1) : std::byte(0));
    }
    
    void writeByte(std::byte b){
        _data.push_back(b);
    }
    
    void rewriteFirstUint32(Uint32 i){
        Uint32 ii = marshall(i);
        const std::byte* bytes = reinterpret_cast<const std::byte*>(&ii);
        for (size_t j = 0; j < sizeof(Uint32); j++) {
            _data[j] = bytes[j];
        }
    }
    
    void writeFloat(float f){
        float ii = marshall(f);
        const std::byte* bytes = reinterpret_cast<const std::byte*>(&ii);
        for (size_t j = 0; j < sizeof(float); j++) {
            _data.push_back(bytes[j]);
        }
    }
    
    void writeSint32(Sint32 i){
        Sint32 ii = marshall(i);
        const std::byte* bytes = reinterpret_cast<const std::byte*>(&ii);
        for (size_t j = 0; j < sizeof(Sint32); j++) {
            _data.push_back(bytes[j]);
        }
    }
    
    void writeUint16(Uint16 i){
        Uint16 ii = marshall(i);
        const std::byte* bytes = reinterpret_cast<const std::byte*>(&ii);
        for (size_t j = 0; j < sizeof(Uint16); j++) {
            _data.push_back(bytes[j]);
        }
    }
    
    void writeUint32(Uint32 i){
        Uint32 ii = marshall(i);
        const std::byte* bytes = reinterpret_cast<const std::byte*>(&ii);
        for (size_t j = 0; j < sizeof(Uint32); j++) {
            _data.push_back(bytes[j]);
        }
    }
    
    void writeUint64(Uint64 i){
        Uint64 ii = marshall(i);
        const std::byte* bytes = reinterpret_cast<const std::byte*>(&ii);
        for (size_t j = 0; j < sizeof(Uint64); j++) {
            _data.push_back(bytes[j]);
        }
    }
    
    const std::vector<std::byte>& serialize() {
        return _data;
    }

    /**
     * Clears the input buffer.
     */
    void reset() {
        _data.clear();
    }
};

class Deserializer{
private:
    /** Currently loaded data */
    std::vector<std::byte> _data;
    /** Position in the data of next byte to read */
    size_t _pos;

public:
    Deserializer() : _pos(0) {}
    
    static std::shared_ptr<Deserializer> alloc() {
        return std::make_shared<Deserializer>();
    }
    
    void receive(const std::vector<std::byte>& msg){
        _data = msg;
        _pos = 0;
    }
    
    bool readBool(){
        if (_pos >= _data.size()) {
            return false;
        }
        uint8_t value = static_cast<uint8_t>(_data[_pos++]);
        return value == 1;
    }
    
    std::byte readByte(){
        if (_pos >= _data.size()) {
            return std::byte(0);
        }
        const std::byte b = _data[_pos++];
        return b;
    }
    
    float readFloat(){
        if (_pos >= _data.size()) {
            return 0.0f;
        }
        const float* r = reinterpret_cast<const float*>(_data.data() + _pos);
        _pos += sizeof(float);
        return marshall(*r);
    }
    
    Sint32 readSint32(){
        if (_pos >= _data.size()) {
            return 0;
        }
        const Sint32* r = reinterpret_cast<const Sint32*>(_data.data() + _pos);
        _pos += sizeof(Sint32);
        return marshall(*r);
    }
    
    Uint16 readUint16(){
        if (_pos >= _data.size()) {
            return 0;
        }
        const Uint16* r = reinterpret_cast<const Uint16*>(_data.data() + _pos);
        _pos += sizeof(Uint16);
        return marshall(*r);
    }
    
    Uint32 readUint32(){
        if (_pos >= _data.size()) {
            return 0;
        }
        const Uint32* r = reinterpret_cast<const Uint32*>(_data.data() + _pos);
        _pos += sizeof(Uint32);
        return marshall(*r);
    }
    
    Uint64 readUint64(){
        if (_pos >= _data.size()) {
            return 0;
        }
        const Uint64* r = reinterpret_cast<const Uint64*>(_data.data() + _pos);
        _pos += sizeof(Uint64);
        return marshall(*r);
    }
    
    void reset(){
        _pos = 0;
        _data.clear();
    }
    
};


#endif /* __RD_NETWORK_DATA_H__ */
