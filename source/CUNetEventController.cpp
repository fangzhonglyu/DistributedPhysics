//
//  CUNetEventController.cpp
//  Networked Physics Library
//
//  Created by Barry Lyu on 9/5/23.
//

#include "CUNetEventController.h"
#include "CUPhysSyncEvent.cpp"

#define MAX_OUT_MSG 10
#define MAX_OUT_BYTES 100000

NetEventController::NetEventController() {
	attachEventType(std::type_index(typeid(PhysSyncEvent)));
}

void NetEventController::updateNet() {
	if (_network) {
		_network->receive([this](const std::string source,
			const std::vector<std::byte>& data) {
				_inEventQueue.push(unwrap(data));
			});
		//checkConnection();
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
}

void NetEventController::attachEventType(const std::type_index& eventType) {
	_eventTypeVector.push_back(eventType);
	_eventTypeMap.insert(std::make_pair(eventType, _eventTypeVector.size() - 1));
}

bool NetEventController::isInEmpty() {
	return _inEventQueue.empty();
}


NetEvent& NetEventController::popInEvent() {
	auto e = _inEventQueue.top();
	_inEventQueue.pop();
	return e;
}

void NetEventController::pushOutEvent(const NetEvent& event) {
	_outEventQueue.push(event);
}
