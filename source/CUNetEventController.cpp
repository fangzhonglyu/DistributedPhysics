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
	attachEventType<PhysSyncEvent>();
}

void NetEventController::updateNet() {
	if (_network) {
		_network->receive([this](const std::string source,
			const std::vector<std::byte>& data) {
				_inEventQueue.push(unwrap(data,source));
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

NetEvent& NetEventController::unwrap(const std::vector<std::byte>& data, std::string source) {
	CUAssertLog(data.size()>0 & (Uint8)data[0] < _eventTypeVector.size(), "Invalid event type");
	std::shared_ptr<NetEvent> e = _eventTypeVector[(Uint8)data[0]]->clone();
	e->setMetaData(0, 0, source);
}

const std::vector<std::byte>& NetEventController::wrap(NetEvent& e) {
	std::vector<std::byte> data;
	data.push_back((std::byte)getType(e));
	auto wrapped = e.serialize();
	data.insert(data.end(), wrapped.begin(), wrapped.end());
	return data;
}
