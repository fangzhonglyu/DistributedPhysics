//
//  RDNetwork.cpp
//  General Purpose Network Controller
//
//  This class handles all network connections.
//
//  Author: Barry Lyu
//  Version: 6/4/23
//

#include "RDNetwork.h"
#include "NetworkData.h"

using namespace cugl;
using namespace cugl::net;

#pragma mark -
#pragma mark Input Controller

NetworkController::NetworkController() :
_status(Status::IDLE),
_roomid(""),
_network(nullptr),
_isHost(true){
}

void NetworkController::dispose() {
	disconnect();
	_assets = nullptr;
	_roomid = "";
}

bool NetworkController::init(const std::shared_ptr<cugl::AssetManager>& assets) {
	_assets = assets;
	auto json = _assets->get<JsonValue>("server");
	_config.set(json);
	_status = Status::IDLE;
	return true;
}

bool NetworkController::connectAsHost() {
	if (_status == Status::NETERROR) {
		disconnect();
	}

	_isHost = true;
	if (_status == Status::IDLE) {
		_status = Status::CONNECTING;
		_network = _network->alloc(_config);
		_network->open();
	}
	return checkConnection();
}

bool NetworkController::connectAsClient(std::string roomid) {
	if (_status == Status::NETERROR) {
		disconnect();
	}

	_isHost = false;
	if (_status == Status::IDLE) {
		_status = Status::CONNECTING;
		_network = _network->alloc(_config, roomid);
		_network->open();
	}
	_roomid = roomid;
	return checkConnection();
}

void NetworkController::disconnect() {
	_network->close();
	_network = nullptr;
	_status = Status::IDLE;
}

bool NetworkController::checkConnection() {
	auto state = _network->getState();
	if (state == NetcodeConnection::State::CONNECTED) {
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

void NetworkController::broadCast(const std::vector<std::byte>&data) {
	if (_status == Status::CONNECTED) {
		_network->broadcast(data);
	}
}

void processData(const std::string source, const std::vector<std::byte>& data) {

}
