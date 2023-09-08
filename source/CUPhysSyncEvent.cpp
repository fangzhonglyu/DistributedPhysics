//
//  CUPhysSyncEvent.cpp
//  Networked Physics Library
//
//  Created by Barry Lyu on 9/5/23.
//

#include "CUNetEvent.h"
#include <unordered_set>

/**
 * The struct for an object snapshot. Contains the object's global Id, position, and velocity.
 */
typedef struct{
    Uint32 objId;
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
    std::unordered_set<Uint32> _objSet;
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

    std::shared_ptr<NetEvent> clone() {
		return std::make_shared<PhysSyncEvent>();
	}
    
    /**
     * This method takes the current list of snapshots and serializes them to a byte vector.
     */
    const std::vector<std::byte>& serialize() override {
        _serializer.reset();
        _serializer.writeUint32((Uint32)_syncList.size());
        for(auto it = _syncList.begin(); it != _syncList.end(); it++){
            ObjParam* obj = &(*it);
            _serializer.writeUint32(obj->objId);
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
            param.objId = _deserializer.readUint32();
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
