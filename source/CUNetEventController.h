//
//  CUNetEventController.h
//  Networked Physics Library
//
//  The header for a general network controllers for multiplayer physics based game
//
//  Created by Barry Lyu on 9/5/23.
//

#ifndef __CU_NET_EVENT_CONTROLLER_H__
#define __CU_NET_EVENT_CONTROLLER_H__
#include <cugl/cugl.h>
#include <unordered_map>
#include <typeindex>
#include <vector>
#include <concepts>
#include <queue>
#include "CUNetEvent.h"
#include "Interpolator.h"

/**
 * This class is a network controller for multiplayer physics based game.
 * 
 * This class holds a {@link NetcodeConnection} and is an extension of the
 * original Network Controller. It is built around an Event-based system that
 * fully encapsulates the network connection. Events across the network are
 * automatically serialized and deserialized.
 * 
 * Connecting to lobbies are encapsulated by {@link connectAsHost()} and
 * {@link connectAsClient()}. When starting a game, the host locks the lobby
 * and calls {@link startGame()} to initiate a handshake, the host distributes
 * shortUID to all player(including host), and players respond by calling
 * {@link markReady()} after they receive the shortUID and finish all game
 * initializations. When host receives responses from all players, the game
 * will officially start and {@link getStatus()} will return INGAME.
 * 
 * Physics synchronization is an optional feature, and is enabled by calling 
 * {@link enablePhysics()}. Upon enabling physics, a dedicated controller is 
 * created to handle physics synchronization. For fine-tuning and more info, 
 * check {@link NetPhysicsController}.
 * 
 * There are three built-in event types: {@link GameStateEvent}, 
 * {@link PhysSyncEvent}, and {@link PhysObjEvent}. See the {@link NetEvent}
 * class and {@link attachEventType()} for how to add and setup custom events.
 */
class NetEventController {
public:
    enum Status {
        /** No connection requested */
        IDLE,
        /** Connecting to lobby */
        CONNECTING,
        /** Connected to lobby */
        CONNECTED,
        /** Handshaking for game start */
        HANDSHAKE,
        /** Ready for game start */
        READY,
        /** Game is in progress */
        INGAME,
        /** Error in connection */
        NETERROR
    };

protected:
    /** The asset manager for the controller. */
    std::shared_ptr<cugl::AssetManager> _assets;
    /** Reference to the App */
    Application* _appRef;
    /** The App fixed-time stamp when the game starts */
    Uint64 _startGameTimeStamp;

    /** The network configuration */
    cugl::net::NetcodeConfig _config;
    /** The network connection */
    std::shared_ptr<cugl::net::NetcodeConnection> _network;

    /** The network controller status */
    Status _status;
    /** The room id of the connected lobby. */
    std::string _roomid;
    /** Whether this device is host */
    bool _isHost;
    /** HOST ONLY, keeps track of ready players during game start handshake. */
    Uint8 _numReady;

    /** Map from attached NetEvent types to uniform event type id */
    std::unordered_map<std::type_index, Uint8> _eventTypeMap;
    /** Vector of NetEvents instances for constructing new events */
    std::vector<std::shared_ptr<NetEvent>> _newEventVector;

    /** Queue for all received custom events. Preserved across updates.*/
    std::queue<std::shared_ptr<NetEvent>> _inEventQueue;
    /** Queue reserved for built-in events */
    std::queue<std::shared_ptr<NetEvent>> _reservedInEventQueue;
    /** Queue for all outbound events. Cleared every update */
    std::vector<std::shared_ptr<NetEvent>> _outEventQueue;
    
    /** Short user id assigned by the host during session */
    Uint32 _shortUID;
    /** Whether physics is enabled. */
    bool _physEnabled;
    /** The physics synchronization controller */
    std::shared_ptr<NetPhysicsController> _physController;

    /* 
     * =================== Note for clarification ===================
     * Outbound events are generated locally and sent to peers.
     * Inbound events are received from peers and processed locally.
     */

    /**
     * Unwraps the a byte vector data into a NetEvent.
     * 
     * The controller automatically detects the type of event, spawns a new 
     * empty instance of that event, and calls the event's 
     * {@link NetEvent#deserialize()} method. This method is only called on 
     * outbound events.
     */
    std::shared_ptr<NetEvent> unwrap(const std::vector<std::byte>& data,std::string source);

    /**
     * Wraps a NetEvent into a byte vector. 
     *
     * The controller calls the event's {@link NetEvent#serialize()} method 
     * and packs the event into byte data. This method is only on inbound 
     * events.
     */
    const std::vector<std::byte> wrap(const std::shared_ptr<NetEvent>& e);
    
    /**
     * Processes all received packets received during the last update.
     * 
     * This method unwraps byte vectors into NetEvents and calls 
     * {@link processReceivedEvent()}.
     */
    void processReceivedData();

    /**
     * Processes all events received during the last update.
     *
     * This method either processes events internally if it is a built-in event
     * and adds them to the inbound event queue otherwise.
     */
    void processReceivedEvent(const std::shared_ptr<NetEvent>& e);

    /**
     * Processes a GameStateEvent.
     *
     * This method updates the controller status based on the event received.
     */
    void processGameStateEvent(const std::shared_ptr<GameStateEvent>& e);

    /**
     * This method checks the connection status and updates the controller status.
     */
    bool checkConnection();
    
    /**
     * Broadcasts all queued outbound events.
     */
    void sendQueuedOutData();
    
    /**
     * Returns the discrete timestamp since the game started. 
     * 
     * Peers should have similar timestamps regardless of when their app was 
     * launched, although peer gameticks might fluctuate due to network 
     * latency. 
     */
    Uint64 getGameTick() const {
        return _appRef->getUpdateCount() - _startGameTimeStamp;
    }

    /**
     * Returns the type id of a NetEvent.
     */
    Uint8 getType(const NetEvent& e) {
        return _eventTypeMap.at(std::type_index(typeid(e)));
    }

    /**
     * Struct for comparing NetEvents by timestamp. Obsolete.
     */
    struct NetEventCompare {
		bool operator()(std::shared_ptr<NetEvent> const& event1, std::shared_ptr<NetEvent> const& event2) {
			return event1->getEventTimeStamp() > event2->getEventTimeStamp();
		}
	};

public:
    /**
     * Constructs a new NetEventController without any initialization.
     */
    NetEventController(void):
        _appRef{ nullptr },
        _status{ Status::IDLE },
        _isHost{ false },
        _startGameTimeStamp{ 0 },
        _shortUID{ 0 },
        _numReady{ 0 },
        _roomid{ "" },
        _assets{ nullptr },
        _network{ nullptr },
        _physController { nullptr },
        _physEnabled{ false }
    {};
    
    /**
     * Initializes the controller with the given asset manager. 
     * 
     * Requires asset to contain key "server" for a json value of the form:
     * {
     *     "lobby" : {
     *         "address" : "xxx.xxx.xxx.xxx",
     *         "port": xxxx
     *     },
     *     "ice servers" : [
     *         {
     *             "turn" : false,
     *             "address" : "xxx.xxx.xxx.xxx",
     *             "port" : xxxx
     *         },
     *         {
     *             "turn" : true,
     *             "address" : "xxx.xxx.xxx.xxx",
     *             "port" : xxxx,
     *             "username": "xxxxxx",
     *             "password": "xxxxxx"
     *         }
     *     ],
     *     "max players" : <INT>,
     *     "API version" : <INT>
     * }

     */
    bool init(const std::shared_ptr<cugl::AssetManager>& assets);

    /**
     * Allocates and initializes a new NetEventController instance. 
     *
     * Returns nullptr if initialization failed.
     */
    static std::shared_ptr<NetEventController> alloc(const std::shared_ptr<cugl::AssetManager>& assets) {
		std::shared_ptr<NetEventController> result = std::make_shared<NetEventController>();
		return (result->init(assets) ? result : nullptr);
	}

    /**
     * Connect to a new lobby as host. 
     *
     * If successful, the controller status changes to CONNECTED, and the
     * {@link _roomid} is set to the lobby id.
     */
    bool connectAsHost();

    /**
     * Connect to an existing lobby as client. 
     *
     * If successful, the controller status changes to CONNECTED.
     */
    bool connectAsClient(std::string roomID);

    /**
     * Disconnect from the current lobby.
     */
    void disconnect();

    /**
     * Enables physics synchronization.
     * 
     * Requires the shortUID to be assigned and the controller.
     * Requires users to handle view changes related to Obstacle creations 
     * and deletions.
     * 
     * @param world The physics world to be synchronized.
     */
    void enablePhysics(std::shared_ptr<cugl::physics2::ObstacleWorld>& world) {
        enablePhysics(world,nullptr);
    }

    /**
     * Enables physics synchronization.
     *
     * Requires the shortUID to be assigned and the controller. 
     * The linkSceneToObsFunc should be a function that links a scene node to 
     * an obstacle with a listener and adds the scene node to a scene graph,
     * typically the addObstacle method in GameScene or analogous class.
     * 
     * @param world The physics world to be synchronized.
     * @param linkSceneToObsFunc Function that links a scene node to an obstacle.
     * 
     */
    void enablePhysics(std::shared_ptr<cugl::physics2::ObstacleWorld>& world, std::function<void(const std::shared_ptr<physics2::Obstacle>&, const std::shared_ptr<scene2::SceneNode>&)> linkSceneToObsFunc) {
        CUAssertLog(_shortUID, "You must receive a UID assigned from host before enabling physics.");
        _physEnabled = true;
        _physController->init(world,_shortUID,linkSceneToObsFunc);
        attachEventType<PhysSyncEvent>();
        attachEventType<PhysObjEvent>();
	}

    /**
     * Disables physics synchronization.
     */
    void disablePhysics() {
		_physEnabled = false;
        _physController = nullptr;
	}

    /**
     * Returns the physics synchronization controller.
     * 
     * Requires physics to be enabled.
     */
    std::shared_ptr<NetPhysicsController> getPhysController() { return _physController; }

    /**
     * Returns the room ID it is currently connected to.
     */
    std::string getRoomID() const { return _roomid; }

    /**
     * Returns whether this device is host. Only valid after connection.
     */
    bool isHost() const {
        return _isHost;
    }

    /**
     * Returns the number of players in lobby. Only valid after connection.
     */
    int getNumPlayers() const {
        if (_network) {
            return (int)(_network->getNumPlayers());
        }
        return 1;
    }

    /**
     * Returns the current status of the controller.
     */
    Status getStatus() const { return _status; }

    /**
     * Returns the shortUID assigned by the host. Only valid after connection.
     * 
     * If the shortUID is 0, the controller didn't receive a UID from the host
     * yet. An assigned shortUID is required for physics synchronization, and 
     * is always non-zero. 
     */
    Uint32 getShortUID() const { return _shortUID; }
    
    /**
     * Starts handshake process for starting game. 
     
     * Once the handshake is finished, the controller changes status to
     * INGAME, starts sending synchronization events if physics is enabled.
     */
    void startGame();

    /**
     * Marks the client as ready for game start. 
     
     * Returns true if the mark was successful. And false otherwise. Only 
     * valid after receiving shortUID from host.
     */
    bool markReady();

    /**
     * Updates the network controller.
     */
    void updateNet();

    /**
     * Attaches a new NetEvent type to the controller. 
     
     * This allows the controller the receive and send custom NetEvent classes.
     * 
     * @param T The event type to be attached, must be a subclass of NetEvent.
     */
    template <typename T>
    void attachEventType() {
        if (!_eventTypeMap.count(std::type_index(typeid(T)))) {
            _eventTypeMap.insert(std::make_pair(std::type_index(typeid(T)), _newEventVector.size()));
            _newEventVector.push_back(std::make_shared<T>());
        }
    }

    /**
     * Returns if there are remaining custom inbound events.
     *
     * Thhe events in this queue is to be polled and processed by outside
     * classes. Inbound events are preserved acrossupdates, and only cleared
     * by {@link popInEvent()}.
     */
    bool isInAvailable();

    /**
     * Returns the next custom inbound event and removes it from the queue.
     * 
     * Requires there to be remaining inbound events.
     */
    std::shared_ptr<NetEvent> popInEvent();

    /**
     * Queues an outbound event to be sent to peers. 
     * 
     * Queued events are sent when {@link updateNet()} is called. and cleared
     * after sending.
     */
    void pushOutEvent(const std::shared_ptr<NetEvent>& e);
};

#endif /* __CU_NET_EVENT_CONTROLLER_H__ */
