//
//  CUNetEventController.cpp
//  Networked Physics Library
//
//  Created by Barry Lyu on 9/5/23.
//

#include "CUNetEventController.h"
#include "CULWSerializer.h"

#define MAX_OUT_MSG 1000
#define MAX_OUT_BYTES 10000000
#define MIN_MSG_LENGTH sizeof(std::byte)+sizeof(Uint64)

using namespace cugl::net;

bool NetEventController::init(const std::shared_ptr<cugl::AssetManager>& assets) {
    // Attach the primitive event types for deserialization
	attachEventType<PhysSyncEvent>(PhysSyncEvent::alloc);
    attachEventType<GameStateEvent>(GameStateEvent::alloc);

    // Configure the NetcodeConnection
    _assets = assets;
    auto json = _assets->get<JsonValue>("server");
    _config.set(json);
    _status = Status::IDLE;
    _appRef = Application::get();
    return true;
}

void NetEventController::startGame() {
    if (_isHost && _status == Status::CONNECTED) {
        _network->startSession();
        auto players = _network->getPlayers();
        Uint8 shortUID = 1;
        for (auto it = players.begin(); it != players.end(); it++) {
            _network->sendTo((*it), wrap(GameStateEvent::allocUIDAssign(shortUID++)));
        }
    }
}

void NetEventController::markReady() {
    if (_status == Status::CONNECTED && _shortUUID != 0) {
		_status = Status::READY;
		_network->broadcast(wrap(GameStateEvent::allocReady()));
	}
}

bool NetEventController::connectAsHost() {
    if (_status == Status::NETERROR) {
        disconnect();
    }

    _isHost = true;
    if (_status == Status::IDLE) {
        _status = Status::CONNECTING;
        _network = NetcodeConnection::alloc(_config);
        _network->open();
    }
    return checkConnection();
}

bool NetEventController::connectAsClient(std::string roomid) {
    if (_status == Status::NETERROR) {
        disconnect();
    }

    _isHost = false;
    if (_status == Status::IDLE) {
        _status = Status::CONNECTING;
        _network = NetcodeConnection::alloc(_config, roomid);
        _network->open();
    }
    _roomid = roomid;
    return checkConnection();
}

void NetEventController::disconnect() {
    _network->close();
    _network = nullptr;
    _shortUUID = 0;
    _status = Status::IDLE;
}

bool NetEventController::checkConnection() {
    auto state = _network->getState();
    if (state == NetcodeConnection::State::CONNECTED) {
        if(_status == Status::CONNECTING || _status == Status::IDLE)
            _status = Status::CONNECTED;
        if (_isHost) {
            _roomid = _network->getRoom();
        }
        return true;
    }
    else if (state == NetcodeConnection::State::NEGOTIATING) {
        _status = Status::CONNECTING;
        return true;
    }
    else if (state == NetcodeConnection::State::DENIED ||
        state == NetcodeConnection::State::DISCONNECTED || state == NetcodeConnection::State::FAILED ||
        state == NetcodeConnection::State::INVALID || state == NetcodeConnection::State::MISMATCHED) {
        _status = Status::NETERROR;
        return false;
    }
    return true;
}

void NetEventController::processReceivedEvent(const std::shared_ptr<NetEvent>& e) {
    if (_status == INGAME){
        if (auto phys = std::dynamic_pointer_cast<PhysSyncEvent>(e)) {
			_physController.processPhysSyncEvent(phys);
            //_reservedInEventQueue.push(phys);
		}
        else {
            _inEventQueue.push(e);
        } 
    }
    else if (auto game = std::dynamic_pointer_cast<GameStateEvent>(e)) {
        processGameStateEvent(game);
    }
}

void NetEventController::processGameStateEvent(const std::shared_ptr<GameStateEvent>& e) {
    if (_status == CONNECTED && e->getType() == GameStateEvent::UID_ASSIGN) {
        _shortUUID = e->getShortUID();
        _status = INSESSION;
    }
    if (_status == READY && e->getType() == GameStateEvent::GAME_START) {
        _status = INGAME;
        _startGameTimeStamp = _appRef->getUpdateCount();
    }
    if (_isHost) {
        if ((_status == INSESSION || _status == READY) && e->getType() == GameStateEvent::CLIENT_RDY) {
            _numReady++;
        }
        if (_status == READY && _numReady == _network->getNumPlayers()) {
            _network->broadcast(wrap(GameStateEvent::allocGameStart()));
        }
    }
}

void NetEventController::processReceivedData(){
    _network->receive([this](const std::string source,
        const std::vector<std::byte>& data) {
        processReceivedEvent(unwrap(data, source)); 
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
        checkConnection();
        processReceivedData();
        
        if (_status == INGAME) {
            if (_isHost) {
                pushOutEvent(_physController.packPhysSync());
            }
			_physController.fixedUpdate();
		}

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
    CUAssertLog(data.size() >= MIN_MSG_LENGTH && (Uint8)data[0] < _allocFuncVector.size(), "Unwrapping invalid event");
    LWDeserializer deserializer;
    deserializer.receive(data);
    Uint8 eventType = (Uint8)deserializer.readByte();
    std::shared_ptr<NetEvent> e = _allocFuncVector[eventType]();
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
