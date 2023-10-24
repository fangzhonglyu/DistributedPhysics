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
    attachEventType<GameStateEvent>();

    // Configure the NetcodeConnection
    _assets = assets;
    auto json = _assets->get<JsonValue>("server");
    _config.set(json);
    _status = Status::IDLE;
    _appRef = Application::get();
    _physController = NetPhysicsController::alloc();
    return true;
}

void NetEventController::startGame() {
    CUAssertLog(_isHost, "Only host should call startGame()");
    if (_status == Status::CONNECTED) {
        _network->startSession();
    }
}

void NetEventController::markReady() {
    if (_status == Status::INSESSION && _shortUID) {
		_status = Status::READY;
		pushOutEvent(GameStateEvent::allocReady());
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
    _shortUID = 0;
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
    else if (_status == CONNECTED && state == NetcodeConnection::State::INSESSION) {
		_status = Status::INSESSION;
        if (_isHost) {
            auto players = _network->getPlayers();
            Uint32 shortUID = 1;
            CULog("NUM Players: %d", players.size());
            for (auto it = players.begin(); it != players.end(); it++) {
                CULog("Player Name: %s", (*it).c_str());
                _network->sendTo((*it), wrap(GameStateEvent::allocUIDAssign(shortUID++)));
            }
        }
		return true;
	}
    else if (_status == READY && _numReady == _network->getNumPlayers() && _isHost) {
        CULog("GAME START MESSAGE SENT");
        pushOutEvent(GameStateEvent::allocGameStart());
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
    if (auto game = std::dynamic_pointer_cast<GameStateEvent>(e)) {
        processGameStateEvent(game);
    } else if (_status == INGAME){
        if (auto phys = std::dynamic_pointer_cast<PhysSyncEvent>(e)) {
            if(_physEnabled)
			    _physController->processPhysSyncEvent(phys);
		}
        else if (auto phys = std::dynamic_pointer_cast<PhysObjEvent>(e)) {
            if (_physEnabled) {
                _physController->processPhysObjEvent(phys);
            }
        }
        else {
            _inEventQueue.push(e);
        } 
    }
}

void NetEventController::processGameStateEvent(const std::shared_ptr<GameStateEvent>& e) {
    CULog("GAME STATE %d, CUR STATE %d", e->getType(), _status);
    if (_status == INSESSION && e->getType() == GameStateEvent::UID_ASSIGN) {
        _shortUID = e->getShortUID();
        CULog("THE UID ASSIGNED IS %x", _shortUID);
    }
    if (_status == READY && e->getType() == GameStateEvent::GAME_START) {
        _status = INGAME;
        _startGameTimeStamp = _appRef->getUpdateCount();
    }
    if (_isHost) {
        if (e->getType() == GameStateEvent::CLIENT_RDY) {
            _numReady++;
            CULog("RECEIVED RDY FROM %s", e->getSourceId().c_str());
        }
    }
    CULog("FINISHED STATE %d", _status);
}

void NetEventController::processReceivedData(){
    _network->receive([this](const std::string source,
        const std::vector<std::byte>& data) {
        CULog("DATA %d, CUR STATE %d, SOURCE %s", data[0], _status, source.c_str());
        processReceivedEvent(unwrap(data, source)); 
    });
}

void NetEventController::sendQueuedOutData(){
    int msgCount = 0;
    int byteCount = 0;
    while (!_outEventQueue.empty()) {
        auto e = _outEventQueue.front();
        auto wrapped = wrap(e);
        msgCount++;
        byteCount += wrapped.size();
        //CULog("flag: %x", (std::byte)getType(*e));
        _network->broadcast(wrap(e));
        _outEventQueue.pop();
    }
}

void NetEventController::updateNet() {
    if(_network){
        checkConnection();
        processReceivedData();

        if (_status == INGAME && _physEnabled) {
            if (_isHost) {
                _physController->packPhysSync();
            }
			_physController->fixedUpdate();
            
            for (auto it = _physController->getOutEvents().begin(); it != _physController->getOutEvents().end(); it++) {
                pushOutEvent(*it);
		    }
            _physController->getOutEvents().clear();
                
		}
        sendQueuedOutData();
    }
}

bool NetEventController::isInAvailable() {
    if ( _inEventQueue.empty() )
        return false;
    std::shared_ptr<NetEvent> top = _inEventQueue.front();
    return top->_eventTimeStamp <= _appRef->getUpdateCount()-_startGameTimeStamp;
}


std::shared_ptr<NetEvent> NetEventController::popInEvent() {
	auto e = _inEventQueue.front();
	_inEventQueue.pop();
	return e;
}

void NetEventController::pushOutEvent(const std::shared_ptr<NetEvent>& e) {
	_outEventQueue.push(e);
}

std::shared_ptr<NetEvent> NetEventController::unwrap(const std::vector<std::byte>& data, std::string source) {
    CUAssertLog(data.size() >= MIN_MSG_LENGTH && (Uint8)data[0] < _newEventVector.size(), "Unwrapping invalid event");
    LWDeserializer deserializer;
    deserializer.receive(data);
    Uint8 eventType = (Uint8)deserializer.readByte();
    std::shared_ptr<NetEvent> e = _newEventVector[eventType]->newEvent();
    Uint64 eventTimeStamp = deserializer.readUint64();
    Uint64 receiveTimeStamp =  _appRef->getUpdateCount()-_startGameTimeStamp;
	e->setMetaData(eventTimeStamp, receiveTimeStamp, source);
    e->deserialize(std::vector(data.begin()+MIN_MSG_LENGTH,data.end()));
    return e;
}

const std::vector<std::byte> NetEventController::wrap(const std::shared_ptr<NetEvent>& e) {
    LWSerializer serializer;
    serializer.writeByte((std::byte)getType(*e));
    //CULog("flag: %x",(std::byte)getType(*e));
    serializer.writeUint64(_appRef->getUpdateCount()-_startGameTimeStamp);
    serializer.writeByteVector(e->serialize());
	return serializer.serialize();
}
