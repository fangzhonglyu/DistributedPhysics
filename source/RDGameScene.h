//
//  RDGameScene.h
//  Rocket Demo
//
//  This is the most important class in this demo.  This class manages the
//  gameplay for this demo.  It also handles collision detection. There is not
//  much to do for collisions; our ObstacleWorld class takes care of all
//  of that for us.  This controller mainly transforms input into gameplay.
//
//  You will notice that we do not use a Scene asset this time.  While we could
//  have done this, we wanted to highlight the issues of connecting physics
//  objects to scene graph objects.  Hence we include all of the API calls.
//
//  WARNING: There are a lot of shortcuts in this design that will do not adapt
//  well to data driven design.  This demo has a lot of simplifications to make
//  it a bit easier to see how everything fits together.  However, the model
//  classes and how they are initialized will need to be changed if you add
//  dynamic level loading.
//
//  This file is based on the CS 3152 PhysicsDemo Lab by Don Holden, 2007
//
//  Author: Walker White
//  Version: 1/10/17
//
#ifndef __RD_GAME_SCENE_H__
#define __RD_GAME_SCENE_H__
#include <cugl/cugl.h>
#include <box2d/b2_world_callbacks.h>
#include <vector>
#include <format>
#include <string>
#include <random>
#include "RDRocketModel.h"
#include "RDInput.h"
#include "NetworkData.h"
#include "RDNetwork.h"
#include "Interpolator.h"
#include "CUNetEventController.h"
/**
 * This class is the primary gameplay constroller for the demo.
 *
 * A world has its own objects, assets, and input controller.  Thus this is
 * really a mini-GameEngine in its own right.  As in 3152, we separate it out
 * so that we can have a separate mode for the loading screen.
 */
class GameScene : public cugl::Scene2 {
protected:
    /** The asset manager for this game mode. */
    std::shared_ptr<cugl::AssetManager> _assets;
    
    // CONTROLLERS
    /** Controller for abstracting out input across multiple platforms */
    RocketInput _input;
    
    // VIEW
    /** Reference to the physics root of the scene graph */
    std::shared_ptr<cugl::scene2::SceneNode> _worldnode;
    /** Reference to the debug root of the scene graph */
    std::shared_ptr<cugl::scene2::SceneNode> _debugnode;
    /** Reference to the win message label */
    std::shared_ptr<cugl::scene2::Label> _winnode;
    
    std::shared_ptr<cugl::scene2::ProgressBar> _chargeBar;

    /** The Box2D world */
    std::shared_ptr<cugl::physics2::ObstacleWorld> _world;
    /** The scale between the physics world and the screen (MUST BE UNIFORM) */
    float _scale;

    // Physics objects for the game
    /** Reference to the goalDoor (for collision detection) */
    std::shared_ptr<cugl::physics2::BoxObstacle> _goalDoor;
    /** Reference to the player1 cannon */
    std::shared_ptr<cugl::scene2::SceneNode> _cannon1Node;
    std::shared_ptr<CannonModel> _cannon1;
    /** Reference to the player2 cannon */
    std::shared_ptr<cugl::scene2::SceneNode> _cannon2Node;
    std::shared_ptr<CannonModel> _cannon2;
    
    /** Host is by default the left cannon */
    bool _isHost;

    /** Whether we have completed this "game" */
    bool _complete;
    /** Whether or not debug mode is active */
    bool _debug;
    
    NetCache _netCache;
    
    NetCache _outCache;
    
    std::shared_ptr<NetEventController> _network;
    
//    cugl::net::NetcodeSerializer _serializer;
//
//    cugl::net::NetcodeDeserializer _deserializer;
    
    Serializer _serializer;

    Deserializer _deserializer;
    
    Uint64 _counter;
    
    Uint64 _bound;
    
    std::shared_ptr<cugl::TextWriter> _writer;
    
    std::mt19937 _rand;
    
    std::queue<Uint32> _objQueue;
    
    std::unordered_map<Uint32,std::shared_ptr<physics2::Obstacle>> _objMap;
    
    std::unordered_set<Uint32> _owned;
    
    std::unordered_map<std::shared_ptr<physics2::Obstacle>,std::shared_ptr<cugl::scene2::SceneNode>> _nodeMap;
    
    std::vector<std::vector<int>> _collisionMap;
    
    Uint32 _nextObj;
    
    NetPhysicsController _itpr;
    
    std::shared_ptr<cugl::physics2::BoxObstacle> _red;
    
#pragma mark Internal Object Management
    
    std::shared_ptr<cugl::physics2::BoxObstacle> addCrateAt(cugl::Vec2 pos, bool original);
    
    /**
     * Lays out the game geography.
     *
     * Pay close attention to how we attach physics objects to a scene graph.
     * The simplest way is to make a subclass, like we do for the rocket.  However,
     * for simple objects you can just use a callback function to lightly couple
     * them.  This is what we do with the crates.
     *
     * This method is really, really long.  In practice, you would replace this
     * with your serialization loader, which would process a level file.
     */
    void populate(bool isInit);
    
    /**
     * Adds the physics object to the physics world and loosely couples it to the scene graph
     *
     * There are two ways to link a physics object to a scene graph node on the
     * screen.  One way is to make a subclass of a physics object, like we did
     * with rocket.  The other is to use callback functions to loosely couple
     * the two.  This function is an example of the latter.
     * the two.  This function is an example of the latter.
     *
     * param obj    The physics object to add
     * param node   The scene graph node to attach it to
     */
    void addObstacle(const std::shared_ptr<cugl::physics2::Obstacle>& obj, 
                     const std::shared_ptr<cugl::scene2::SceneNode>& node);
    
    void addObstacleAlt(const std::shared_ptr<cugl::physics2::Obstacle>& obj,
                     const std::shared_ptr<cugl::scene2::SceneNode>& node);

    /**
     * Returns the active screen size of this scene.
     *
     * This method is for graceful handling of different aspect
     * ratios
     */
    cugl::Size computeActiveSize() const;
    
private:
    /**
     * Processes data sent over the network.
     *
     * Once connection is established, all data sent over the network consistes of
     * byte vectors. This function is a call back function to process that data.
     * Note that this function may be called *multiple times* per animation frame,
     * as the messages can come from several sources.
     *
     * Typically this is where players would communicate their names after being
     * connected. In this lab, we only need it to do one thing: communicate that
     * the host has started the game.
     *
     * @param source    The UUID of the sender
     * @param data      The data received
     */
    void processData(const std::string source, const std::vector<std::byte>& data);
    
    void updateNet();
    
    void transmitNetdata(const netdata data);
    
    void queueNetdata(netdata data);
    
public:
#pragma mark -
#pragma mark Constructors
    /**
     * Creates a new game world with the default values.
     *
     * This constructor does not allocate any objects or start the controller.
     * This allows us to use a controller without a heap pointer.
     */
    GameScene();
    
    /**
     * Disposes of all (non-static) resources allocated to this mode.
     *
     * This method is different from dispose() in that it ALSO shuts off any
     * static resources, like the input controller.
     */
    ~GameScene() { dispose(); }
    
    /**
     * Disposes of all (non-static) resources allocated to this mode.
     */
    void dispose();
    
    /**
     * Initializes the controller contents, and starts the game
     *
     * The constructor does not allocate any objects or memory.  This allows
     * us to have a non-pointer reference to this controller, reducing our
     * memory allocation.  Instead, allocation happens in this method.
     *
     * The game world is scaled so that the screen coordinates do not agree
     * with the Box2d coordinates.  This initializer uses the default scale.
     *
     * @param assets    The (loaded) assets for this game mode
     *
     * @return true if the controller is initialized properly, false otherwise.
     */
    bool init(const std::shared_ptr<cugl::AssetManager>& assets, const std::shared_ptr<NetEventController> network);

    /**
     * Initializes the controller contents, and starts the game
     *
     * The constructor does not allocate any objects or memory.  This allows
     * us to have a non-pointer reference to this controller, reducing our
     * memory allocation.  Instead, allocation happens in this method.
     *
     * The game world is scaled so that the screen coordinates do not agree
     * with the Box2d coordinates.  The bounds are in terms of the Box2d
     * world, not the screen.
     *
     * @param assets    The (loaded) assets for this game mode
     * @param rect      The game bounds in Box2d coordinates
     *
     * @return  true if the controller is initialized properly, false otherwise.
     */
    bool init(const std::shared_ptr<cugl::AssetManager>& assets, const cugl::Rect rect, const std::shared_ptr<NetEventController> network);
    
    /**
     * Initializes the controller contents, and starts the game
     *
     * The constructor does not allocate any objects or memory.  This allows
     * us to have a non-pointer reference to this controller, reducing our
     * memory allocation.  Instead, allocation happens in this method.
     *
     * The game world is scaled so that the screen coordinates do not agree
     * with the Box2d coordinates.  The bounds are in terms of the Box2d
     * world, not the screen.
     *
     * @param assets    The (loaded) assets for this game mode
     * @param rect      The game bounds in Box2d coordinates
     * @param gravity   The gravitational force on this Box2d world
     *
     * @return  true if the controller is initialized properly, false otherwise.
     */
    bool init(const std::shared_ptr<cugl::AssetManager>& assets, const cugl::Rect rect, const cugl::Vec2 gravity, const std::shared_ptr<NetEventController> network);
    
    void setHost(bool isHost);
    
    
#pragma mark -
#pragma mark State Access
    /**
     * Returns true if the gameplay controller is currently active
     *
     * @return true if the gameplay controller is currently active
     */
    bool isActive( ) const { return _active; }

    /**
     * Returns true if debug mode is active.
     *
     * If true, all objects will display their physics bodies.
     *
     * @return true if debug mode is active.
     */
    bool isDebug( ) const { return _debug; }
    
    /**
     * Sets whether debug mode is active.
     *
     * If true, all objects will display their physics bodies.
     *
     * @param value whether debug mode is active.
     */
    void setDebug(bool value) { _debug = value; _debugnode->setVisible(value); }
    
    /**
     * Returns true if the level is completed.
     *
     * If true, the level will advance after a countdown
     *
     * @return true if the level is completed.
     */
    bool isComplete( ) const { return _complete; }
    
    /**
     * Sets whether the level is completed.
     *
     * If true, the level will advance after a countdown
     *
     * @param value whether the level is completed.
     */
    void setComplete(bool value) { _complete = value; _winnode->setVisible(value); }
    
    
#pragma mark -
#pragma mark Gameplay Handling
#if USING_PHYSICS
    virtual void preUpdate(float timestep);
    virtual void postUpdate(float timestep);
    virtual void fixedUpdate();
    
    void logData();

#else
    /**
     * The method called to update the game mode.
     *
     * This method contains any gameplay code that is not an OpenGL call.
     *
     * @param timestep  The amount of time (in seconds) since the last frame
     */
    void update(float timestep);
#endif

    /**
     * Resets the status of the game so that we can play again.
     */
    void reset();
    
    /**
     * Packs a fire input message
     */
    netdata packFire(Uint64 timestamp);
    
    netdata packFire(Uint64 timestamp,float power);
    
    std::shared_ptr<PhysSyncEvent> packState(Uint64 timestamp);
    
    netdata packReset(Uint64 timestamp);
    
    netdata packCannon(Uint64 timestamp);
    
    void processCannon(netdata data);
    
    void processFire(netdata data);
    
    void processState(netdata data);
    
    void processCache();
    
#pragma mark -
#pragma mark Collision Handling
    /**
     * Processes the start of a collision
     *
     * This method is called when we first get a collision between two objects. 
     * We use this method to test if it is the "right" kind of collision.  In 
     * particular, we use it to test if we make it to the win door.
     *
     * @param  contact  The two bodies that collided
     */
    void beginContact(b2Contact* contact);

    /**
     * Handles any modifications necessary before collision resolution
     *
     * This method is called just before Box2D resolves a collision.  We use 
     * this method to implement sound on contact, using the algorithms outlined 
     * in Ian Parberry's "Introduction to Game Physics with Box2D".
     *
     * @param  contact  The two bodies that collided
     * @param  contact  The collision manifold before contact
     */
    void beforeSolve(b2Contact* contact, const b2Manifold* oldManifold);
};

#endif /* __RD_GAME_MODE_H__ */
