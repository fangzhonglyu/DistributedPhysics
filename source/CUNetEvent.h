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
#include "CULWSerializer.h"

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
    
    virtual std::shared_ptr<NetEvent> newEvent(){
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

#define UID_ASSIGN_FLAG  100
#define CLIENT_RDY_FLAG  101
#define GAME_START_FLAG  102
#define GAME_RESET_FLAG  103
#define GAME_PAUSE_FLAG  104
#define GAME_RESUME_FLAG 105

/**
 * This class represents a message for the networked physics library to notify of game state changes, such as start game, reset, or pause.
 */
class GameStateEvent : public NetEvent {
public:
    enum Type
    {
        UID_ASSIGN,
        CLIENT_RDY,
        GAME_START,
        GAME_RESET,
        GAME_PAUSE,
        GAME_RESUME
    };
protected:
    Type _type;
    Uint32 _shortUID;

public:
    static std::shared_ptr<GameStateEvent> alloc() {
        return std::make_shared<GameStateEvent>();
    }
    
    std::shared_ptr<NetEvent> newEvent() override {
        return std::make_shared<GameStateEvent>();
    }

    static std::shared_ptr<NetEvent> allocGameStart() {
        std::shared_ptr<GameStateEvent> ptr = std::make_shared<GameStateEvent>();
        ptr->setType(GAME_START);
        return ptr;
    }

    static std::shared_ptr<NetEvent> allocReady() {
        std::shared_ptr<GameStateEvent> ptr = std::make_shared<GameStateEvent>();
        ptr->setType(CLIENT_RDY);
        return ptr;
    }

    static std::shared_ptr<NetEvent> allocUIDAssign(Uint32 _shortUID) {
        std::shared_ptr<GameStateEvent> ptr = std::make_shared<GameStateEvent>();
        ptr->setType(UID_ASSIGN);
        ptr->_shortUID = _shortUID;
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
            case CLIENT_RDY:
                data.push_back(std::byte(CLIENT_RDY_FLAG));
                break;
            case UID_ASSIGN:
				data.push_back(std::byte(UID_ASSIGN_FLAG));
                data.push_back(std::byte(_shortUID));
				break;
            default:
                CUAssertLog(false, "Serializing invalid game state event type");
        }
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
                break;
            case CLIENT_RDY_FLAG:
				_type = CLIENT_RDY;
				break;
            case UID_ASSIGN_FLAG:
                _type = UID_ASSIGN;
                _shortUID = (Uint8)data[1];
                break;
            default:
                CUAssertLog(false, "Deserializing game state event type");
        }
    }
};

/**
 * The struct for an object snapshot. Contains the object's global Id, position, and velocity.
 */
typedef struct{
    Uint64 objId;
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
    std::unordered_set<Uint64> _objSet;
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
    void addObj(const std::shared_ptr<physics2::Obstacle>& obj, Uint64 id){
        if(_objSet.count(id))
            return;
        
        _objSet.insert(id);
        ObjParam param;
        param.objId = id;
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

    static std::shared_ptr<PhysSyncEvent> alloc() {
        return std::make_shared<PhysSyncEvent>();
    }
    
    std::shared_ptr<NetEvent> newEvent() override{
        return std::make_shared<PhysSyncEvent>();
    }
    
    /**
     * This method takes the current list of snapshots and serializes them to a byte vector.
     */
    std::vector<std::byte> serialize() override {
        _serializer.reset();
        _serializer.writeUint64((Uint64)_syncList.size());
        for(auto it = _syncList.begin(); it != _syncList.end(); it++){
            ObjParam& obj = (*it);
            _serializer.writeUint64(obj.objId);
            _serializer.writeFloat(obj.x);
            _serializer.writeFloat(obj.y);
            _serializer.writeFloat(obj.vx);
            _serializer.writeFloat(obj.vy);
            _serializer.writeFloat(obj.angle);
            _serializer.writeFloat(obj.vAngular);
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
        Uint64 numObjs = _deserializer.readUint64();
        for(size_t i = 0; i < numObjs; i++){
            ObjParam param;
            param.objId = _deserializer.readUint64();
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

class ObstacleFactory {
public:
    static std::shared_ptr<ObstacleFactory> alloc() {
        return std::make_shared<ObstacleFactory>();
    };

    /**
     * Serialize any paramater that the event contains to a vector of bytes.
     */
    virtual std::pair<std::shared_ptr<physics2::Obstacle>, std::shared_ptr<scene2::SceneNode>> createObstacle(const std::vector<std::byte>& params) {
        return std::make_pair(std::make_shared<physics2::BoxObstacle>(), std::make_shared<scene2::SceneNode>());
    }
};

#define OBJ_CREATION_FLAG 200
#define OBJ_DELETION_FLAG 201

class PhysObjEvent : public NetEvent{
public:
    enum Type
    {
        OBJ_CREATION,
        OBJ_DELETION
    };

protected:
    Type _type;
    Uint32 _obstacleFactId;
    Uint64 _objId;
    std::shared_ptr<std::vector<std::byte>> _packedParam;

    LWSerializer _serializer;
    LWDeserializer _deserializer;

    static std::vector<std::shared_ptr<ObstacleFactory>> _obstacleFacts;

public:

    Uint32 getObstacleFactId() const { return _obstacleFactId; }

    Uint64 getObjId() const { return _objId; }

    const std::shared_ptr<std::vector<std::byte>> getPackedParam() const { return _packedParam; }
    
    void initCreation(Uint32 obstacleFactId, Uint64 objId, std::shared_ptr<std::vector<std::byte>> packedParam){
        _type = OBJ_CREATION;
        _obstacleFactId = obstacleFactId;
        _objId = objId;
        _packedParam = packedParam;
    }
    
    void initDeletion(Uint64 objId){
        _type = OBJ_DELETION;
        _objId = objId;
    }
    
    static std::shared_ptr<PhysObjEvent> allocCreation(Uint32 obstacleFactId, Uint64 objId, std::shared_ptr<std::vector<std::byte>> packedParam){
        auto e = std::make_shared<PhysObjEvent>();
        e->initCreation(obstacleFactId, objId, packedParam);
        return e;
    }
    
    static std::shared_ptr<PhysObjEvent> allocDeletion(Uint64 objId){
        auto e = std::make_shared<PhysObjEvent>();
        e->initDeletion(objId);
        return e;
    }
    
    std::shared_ptr<NetEvent> newEvent() override {
        return std::make_shared<PhysObjEvent>();
    }

    std::vector<std::byte> serialize() override {
        _serializer.reset();
        _serializer.writeUint32(_obstacleFactId);
        _serializer.writeUint64(_objId);
        _serializer.writeByteVector(*_packedParam);
        return _serializer.serialize();
    }

    void deserialize(const std::vector<std::byte>& data) override {
        if (data.size() < sizeof(Uint32)+sizeof(Uint64))
            return;
        _deserializer.reset();
        _deserializer.receive(data);
        _obstacleFactId = _deserializer.readUint32();
        _objId = _deserializer.readUint64();
        _packedParam = std::make_shared<std::vector<std::byte>>(data.begin() + sizeof(Uint32) + sizeof(Uint64),data.end());
    }

};


#endif /* __CU_NET_EVENT_H__ */
