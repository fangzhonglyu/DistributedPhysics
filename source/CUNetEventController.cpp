//
//  CUNetEventController.cpp
//  Networked Physics Library
//
//  Created by Barry Lyu on 9/5/23.
//

#include "CUNetEventController.h"
#include "CULWSerializer.h"

#define MAX_OUT_MSG 10
#define MAX_OUT_BYTES 100000
#define MIN_MSG_LENGTH sizeof(std::byte)+sizeof(Uint64)

#define GAME_START_EVENT

bool NetEventController::init(const std::shared_ptr<cugl::AssetManager>& assets) {
    // Attach the primitive event types for deserialization
	attachEventType<PhysSyncEvent>();
    attachEventType<GameStateEvent>();

    // Configure the NetcodeConnection
    _assets = assets;
    auto json = _assets->get<JsonValue>("server");
    _config.set(json);
    _status = Status::IDLE;
    return true;
}

void NetEventController::startGame() {
    if (_isHost) {
        pushOutEvent(GameStateEvent::alloc(GameStateEvent::GAME_START));
        sendQueuedOutData();
    }
    _startGameTimeStamp = _appRef->getUpdateCount();
}

void NetEventController::processReceivedData(){
    _network->receive([this](const std::string source,
        const std::vector<std::byte>& data) {
        _inEventQueue.push(unwrap(data,source));
    });
}

void NetEventController::sendQueuedOutData(){
    int msgCount = 0;
    int byteCount = 0;
    while (!_outEventQueue.empty()) {
        auto e = _outEventQueue.top();
        auto wrapped = wrap(e);
        msgCount++;
        byteCount += wrapped.size();
        if (msgCount > MAX_OUT_MSG || byteCount > MAX_OUT_BYTES) {
            break;
        }
        else {
            _network->broadcast(wrap(e));
            _outEventQueue.pop();
        }
    }
}

void NetEventController::updateNet() {
    if(_network){
        processReceivedData();
        sendQueuedOutData();
    }
}

bool NetEventController::isInAvailable() {
    if ( _inEventQueue.empty() )
        return false;
    std::shared_ptr<NetEvent> top = _inEventQueue.top();
    return top->_eventTimeStamp <= _appRef->getUpdateCount()-_startGameTimeStamp;
}


std::shared_ptr<NetEvent> NetEventController::popInEvent() {
	auto e = _inEventQueue.top();
	_inEventQueue.pop();
	return e;
}

void NetEventController::pushOutEvent(const std::shared_ptr<NetEvent>& e) {
	_outEventQueue.push(e);
}

std::shared_ptr<NetEvent> NetEventController::unwrap(const std::vector<std::byte>& data, std::string source) {
    CUAssertLog(data.size() >= MIN_MSG_LENGTH && (Uint8)data[0] < _eventTypeVector.size(), "Unwrapping invalid event");
    LWDeserializer deserializer;
    deserializer.receive(data);
    Uint8 eventType = (Uint8)deserializer.readByte();
    std::shared_ptr<NetEvent> e = _eventTypeVector[eventType]->alloc();
    Uint64 eventTimeStamp = deserializer.readUint64();
    Uint64 receiveTimeStamp =  _appRef->getUpdateCount()-_startGameTimeStamp;
	e->setMetaData(eventTimeStamp, receiveTimeStamp, source);
    e->deserialize(std::vector(data.begin()+MIN_MSG_LENGTH,data.end()));
    return e;
}

const std::vector<std::byte>& NetEventController::wrap(const std::shared_ptr<NetEvent>& e) {
    LWSerializer serializer;
    serializer.writeByte((std::byte)getType(*e));
    serializer.writeUint64(_appRef->getUpdateCount()-_startGameTimeStamp);
    serializer.writeByteVector(e->serialize());
	return serializer.serialize();
}
