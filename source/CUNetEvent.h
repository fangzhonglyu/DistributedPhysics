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
    static std::shared_ptr<NetEvent> alloc() {
        return std::make_shared<NetEvent>();
    };
    /**
     * Serialize any paramater that the event contains to a vector of bytes.
     */
    virtual std::vector<std::byte> serialize(){
        return std::vector<std::byte>();
    };
    /**
     * Deserialize a vector of bytes and set the corresponding parameters.
     *
     * @param data  a byte vector packed by serialize()
     *
     * This function should be the "reverse" of the serialize() function: it should be able to recreate a serialized event entirely, setting all the useful parameters of this class.
     */
    virtual void deserialize(const std::vector<std::byte>& data) { }
    
    Uint64 getEventTimeStamp() const { return _eventTimeStamp; }
    
    Uint64 getReceiveTimeStamp() const { return _receiveTimeStamp; }
    
    const std::string getSourceId() const { return _sourceID; }   
};

#define GAME_START_FLAG  100
#define GAME_RESET_FLAG  101
#define GAME_PAUSE_FLAG  102
#define GAME_RESUME_FLAG 103

/**
 * This class represents a message for the networked physics library to notify of game state changes, such as start game, reset, or pause.
 */
class GameStateEvent : public NetEvent {
public:
    enum Type
    {
        GAME_START,
        GAME_RESET,
        GAME_PAUSE,
        GAME_RESUME
    };
protected:
    Type _type;
    Uint8 _shortUID;

public:
    static std::shared_ptr<NetEvent> alloc() {
        return std::make_shared<GameStateEvent>();
    }

    static std::shared_ptr<NetEvent> alloc(Type t) {
        std::shared_ptr<GameStateEvent> ptr = std::make_shared<GameStateEvent>();
        ptr->setType(t);
        return ptr;
    }

    static std::shared_ptr<NetEvent> alloc(Type t, Uint32 _shortUID) {
        std::shared_ptr<GameStateEvent> ptr = std::make_shared<GameStateEvent>();
        ptr->setType(t);
        return ptr;
    }

    GameStateEvent() {
        _type = GAME_START;
    }

    GameStateEvent(Type t) {
        _type = t;
    }

    void setType(Type t) {
        _type = t;
    }

    Type getType() const {
        return _type;
    }

    Uint8 getShortUID() const {
        return _shortUID;
    }

    /**
     * This method takes the current list of snapshots and serializes them to a byte vector.
     */
    std::vector<std::byte> serialize() override {
        std::vector<std::byte> data;
        switch (_type) {
            case GAME_START:
                data.push_back(std::byte(GAME_START_FLAG));
                break;
            case GAME_RESET:
                data.push_back(std::byte(GAME_RESET_FLAG));
                break;
            case GAME_PAUSE:
                data.push_back(std::byte(GAME_PAUSE_FLAG));
                break;
            case GAME_RESUME:
                data.push_back(std::byte(GAME_RESUME_FLAG));
                break;
        }
        data.push_back(std::byte(_shortUID));
        return data;
    }

    /**
     * This method unpacks a byte vector to a list of snapshots that can be read and used for physics synchronizations.
     */
    void deserialize(const std::vector<std::byte>& data) override {
        Uint8 flag = (Uint8)data[0];
        switch (flag) {
            case GAME_START_FLAG:
                _type = GAME_START;
                break;
            case GAME_RESET_FLAG:
                _type = GAME_RESET;
                break;
            case GAME_PAUSE_FLAG:
                _type = GAME_PAUSE;
                break;
            case GAME_RESUME_FLAG:
                _type = GAME_RESUME;
        }
        _shortUID = (Uint8)data[1];
    }
};

/**
 * The struct for an object snapshot. Contains the object's global Id, position, and velocity.
 */
typedef struct{
    std::string objId;
    float x;
    float y;
    float vx;
    float vy;
    float angle;
    float vAngular;
} ObjParam;

/**
 * This class represents a message for the networked physics library to synchronize object positions. It should only be used by the networked physics library, not for custom game informations.
 */
class PhysSyncEvent : public NetEvent{
private:
    /** The set of objectIds of all objects added to be serialized. Used to prevent duplicate objects. */
    std::unordered_set<std::string> _objSet;
    /** The serializer for converting basic types to byte vectors. */
    net::NetcodeSerializer _serializer;
    /** The deserializer for converting byte vectors to basic types. */
    net::NetcodeDeserializer _deserializer;
protected:
    /** The vector of added object snapshots. */
    std::vector<ObjParam> _syncList;

public:
    /**
     * This method takes a snapshot of an obstacle's current position and velocity, and adds the snapshot to the list for serialization.
     *
     * @param obj the obstacle reference to add, duplicate obstacles would be ignored
     */
    void addObj(std::shared_ptr<physics2::Obstacle>& obj){
        if(_objSet.count(obj->_id))
            return;
        
        _objSet.insert(obj->_id);
        ObjParam param;
        param.objId = obj->_id;
        param.x = obj->getX();
        param.y = obj->getY();
        param.vx = obj->getVX();
        param.vy = obj->getVY();
        param.angle = obj->getAngle();
        param.vAngular = obj->getAngularVelocity();
        _syncList.push_back(param);
    }

    const std::vector<ObjParam>& getSyncList() const {
		return _syncList;
	}

    static std::shared_ptr<NetEvent> alloc() {
        return std::make_shared<PhysSyncEvent>();
    }
    
    /**
     * This method takes the current list of snapshots and serializes them to a byte vector.
     */
    std::vector<std::byte> serialize() override {
        _serializer.reset();
        _serializer.writeUint32((Uint32)_syncList.size());
        for(auto it = _syncList.begin(); it != _syncList.end(); it++){
            ObjParam* obj = &(*it);
            _serializer.writeString(obj->objId);
            _serializer.writeFloat(obj->x);
            _serializer.writeFloat(obj->y);
            _serializer.writeFloat(obj->vx);
            _serializer.writeFloat(obj->vy);
            _serializer.writeFloat(obj->angle);
            _serializer.writeFloat(obj->vAngular);
        }
        return _serializer.serialize();
    }
    
    /**
     * This method unpacks a byte vector to a list of snapshots that can be read and used for physics synchronizations.
     */
    void deserialize(const std::vector<std::byte> &data) override {
        if(data.size() < 4)
            return;
        
        _deserializer.reset();
        _deserializer.receive(data);
        Uint32 numObjs = _deserializer.readUint32();
        for(size_t i = 0; i < numObjs; i++){
            ObjParam param;
            param.objId = _deserializer.readString();
            param.x = _deserializer.readFloat();
            param.y = _deserializer.readFloat();
            param.vx = _deserializer.readFloat();
            param.vy = _deserializer.readFloat();
            param.angle = _deserializer.readFloat();
            param.vAngular = _deserializer.readFloat();
            _syncList.push_back(param);
        }
    }
};



#endif /* __CU_NET_EVENT_H__ */
