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
                data.push_back(std::byte(GAME_START));
                break;
            case GAME_RESET:
                data.push_back(std::byte(GAME_RESET));
                break;
            case GAME_PAUSE:
                data.push_back(std::byte(GAME_PAUSE));
                break;
            case GAME_RESUME:
                data.push_back(std::byte(GAME_RESUME));
                break;
            case CLIENT_RDY:
                data.push_back(std::byte(CLIENT_RDY));
                break;
            case UID_ASSIGN:
				data.push_back(std::byte(UID_ASSIGN));
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
        Type flag = (Type)data[0];
        switch (flag) {
            case GAME_START:
                _type = GAME_START;
                break;
            case GAME_RESET:
                _type = GAME_RESET;
                break;
            case GAME_PAUSE:
                _type = GAME_PAUSE;
                break;
            case GAME_RESUME:
                _type = GAME_RESUME;
                break;
            case CLIENT_RDY:
				_type = CLIENT_RDY;
				break;
            case UID_ASSIGN:
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

class PhysObjEvent : public NetEvent{
public:
    enum Type
    {
        OBJ_CREATION,
        OBJ_DELETION,
        OBJ_BODY_TYPE,
        OBJ_POSITION,
        OBJ_VELOCITY,
        OBJ_ANGLE,
        OBJ_ANGULAR_VEL,
        OBJ_BOOL_CONSTS,
        OBJ_FLOAT_CONSTS
    };

protected:
    Type _type;
    Uint32 _obstacleFactId;
    Uint64 _objId;
    std::shared_ptr<std::vector<std::byte>> _packedParam;

public:
    Vec2 _pos;
    Vec2 _vel;
    
	float _angle;
    float _angularVel;

    //bool consts
    bool _isStatic;
    bool _isEnabled;
    bool _isAwake;
    bool _isSleepingAllowed;
    bool _isFixedRotation;
    bool _isBullet;
    bool _isSensor;

    //float consts
	float _density;
    float _friction;
    float _restitution;
    float _linearDamping;
    float _angularDamping;
    float _gravityScale;
    float _mass;
    float _inertia;
    Vec2 _centroid;

    b2BodyType _bodyType;

    LWSerializer _serializer;
    LWDeserializer _deserializer;

    static std::vector<std::shared_ptr<ObstacleFactory>> _obstacleFacts;

public:

    Uint32 getObstacleFactId() const { return _obstacleFactId; }

    Uint64 getObjId() const { return _objId; }

    Type getType() const { return _type; }

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

    void initPos(Uint64 objId, Vec2 pos) {
        _type = OBJ_POSITION;
        _objId = objId;
		_pos = pos;
    }

    void initVel(Uint64 objId, Vec2 vel) {
        _type = OBJ_VELOCITY;
        _objId = objId;
        _vel = vel;
    }

    void initAngle(Uint64 objId, float angle) {
		_type = OBJ_ANGLE;
		_objId = objId;
		_angle = angle;
	}

    void initAngularVel(Uint64 objId, float angularVel) {
		_type = OBJ_ANGULAR_VEL;
		_objId = objId;
		_angularVel = angularVel;
	}

    void initBodyType(Uint64 objId, b2BodyType bodyType) {
        _type = OBJ_BODY_TYPE;
        _objId = objId;
        _bodyType = bodyType;
    }

    void initBoolConsts(Uint64 objId, bool isEnabled, bool isAwake, bool isSleepingAllowed, bool isFixedRotation, bool isBullet, bool isSensor) {
		_type = OBJ_BOOL_CONSTS;
		_objId = objId;
		_isEnabled = isEnabled;
		_isAwake = isAwake;
		_isSleepingAllowed = isSleepingAllowed;
		_isFixedRotation = isFixedRotation;
		_isBullet = isBullet;
		_isSensor = isSensor;
	}

    void initFloatConsts(Uint64 objId, float density, float friction, float restitution, float linearDamping, float angularDamping, float gravityScale) {
        _type = OBJ_FLOAT_CONSTS;
        _objId = objId;
        _density = density;
        _friction = friction;
        _restitution = restitution;
        _linearDamping = linearDamping;
        _angularDamping = angularDamping;
        _gravityScale = gravityScale;
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
        _serializer.writeUint32((uint32)_type);
        _serializer.writeUint64(_objId);
        switch (_type){
            case PhysObjEvent::OBJ_CREATION:
                _serializer.writeUint32(_obstacleFactId);
                _serializer.writeByteVector(*_packedParam);
                break;
            case PhysObjEvent::OBJ_DELETION:
                break;
            case PhysObjEvent::OBJ_BODY_TYPE:
                _serializer.writeUint32(_bodyType);
                break;
            case PhysObjEvent::OBJ_POSITION:
                _serializer.writeFloat(_pos.x);
			    _serializer.writeFloat(_pos.y);
			    break;
            case PhysObjEvent::OBJ_VELOCITY:
			    _serializer.writeFloat(_vel.x);
                _serializer.writeFloat(_vel.y);
                break;
            case PhysObjEvent::OBJ_ANGLE:
				_serializer.writeFloat(_angle);
				break;
            case PhysObjEvent::OBJ_ANGULAR_VEL:
                _serializer.writeFloat(_angularVel);
                break;
            case PhysObjEvent::OBJ_BOOL_CONSTS:
                _serializer.writeBool(_isEnabled);
                _serializer.writeBool(_isAwake);
                _serializer.writeBool(_isSleepingAllowed);
                _serializer.writeBool(_isFixedRotation);
                _serializer.writeBool(_isBullet);
                _serializer.writeBool(_isSensor);
				break;
			case PhysObjEvent::OBJ_FLOAT_CONSTS:
				_serializer.writeFloat(_density);
				_serializer.writeFloat(_friction);
				_serializer.writeFloat(_restitution);
				_serializer.writeFloat(_linearDamping);
				_serializer.writeFloat(_angularDamping);
				_serializer.writeFloat(_gravityScale);
                _serializer.writeFloat(_mass);
                _serializer.writeFloat(_inertia);
                _serializer.writeFloat(_centroid.x);
                _serializer.writeFloat(_centroid.y);
				break;
            default:
				CUAssertLog(false, "Serializing invalid obstacle event type");
        }
        return _serializer.serialize();
    }

    void deserialize(const std::vector<std::byte>& data) override {
        if (data.size() < sizeof(Uint32)+sizeof(Uint64))
            return;
        _deserializer.reset();
        _deserializer.receive(data);
        _type = (Type)_deserializer.readUint32();
        _objId = _deserializer.readUint64();
        switch (_type) {
        case PhysObjEvent::OBJ_CREATION:
            _obstacleFactId = _deserializer.readUint32();
            _packedParam = std::make_shared<std::vector<std::byte>>(data.begin() + 2 * sizeof(Uint32) + sizeof(Uint64), data.end());
            break;
        case PhysObjEvent::OBJ_DELETION:
			break;
        case PhysObjEvent::OBJ_BODY_TYPE:
			 _bodyType = (b2BodyType)_deserializer.readUint32();
			 break;
        case PhysObjEvent::OBJ_POSITION:
            _pos.x = _deserializer.readFloat();
            _pos.y = _deserializer.readFloat();
            break;
        case PhysObjEvent::OBJ_VELOCITY:
            _vel.x = _deserializer.readFloat();
            _vel.y = _deserializer.readFloat();
            break;
        case PhysObjEvent::OBJ_ANGLE:
            _angle = _deserializer.readFloat();
            break;
        case PhysObjEvent::OBJ_ANGULAR_VEL:
            _angularVel = _deserializer.readFloat();
            break;
        case PhysObjEvent::OBJ_BOOL_CONSTS:
            _isEnabled = _deserializer.readBool();
            _isAwake = _deserializer.readBool();
            _isSleepingAllowed = _deserializer.readBool();
            _isFixedRotation = _deserializer.readBool();
            _isBullet = _deserializer.readBool();
            _isSensor = _deserializer.readBool();
            break;
        case PhysObjEvent::OBJ_FLOAT_CONSTS:
            _density = _deserializer.readFloat();
            _friction = _deserializer.readFloat();
            _restitution = _deserializer.readFloat();
            _linearDamping = _deserializer.readFloat();
            _angularDamping = _deserializer.readFloat();
            _gravityScale = _deserializer.readFloat();
            _mass = _deserializer.readFloat();
            _inertia = _deserializer.readFloat();
            _centroid.x = _deserializer.readFloat();
            _centroid.y = _deserializer.readFloat();
            break;
        default:
            CUAssertLog(false, "Deserializing invalid obstacle event type");
        }
    }

};


#endif /* __CU_NET_EVENT_H__ */
