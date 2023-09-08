//
//  CUGameStateEvent.cpp
//  Networked Physics Library
//
//  Created by Barry Lyu on 9/5/23.
//

#include "CUNetEvent.h"

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


public:
    std::shared_ptr<NetEvent> clone() override {
        return std::make_shared<GameStateEvent>();
    }

    static std::shared_ptr<NetEvent> alloc(Type t) {
        std::shared_ptr<GameStateEvent> ptr = std::make_shared<GameStateEvent>();
        ptr->setType(t);
        return ptr;
	}

    GameStateEvent() {}

    GameStateEvent(Type t) {
        _type = t;
	}

    void setType(Type t) {
		_type = t;
	}

    Type getType() {
        return _type;
    }

    /**
     * This method takes the current list of snapshots and serializes them to a byte vector.
     */
    const std::vector<std::byte>& serialize() override {
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
        data.push_back(std::byte(GAME_START));
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
    }
};
