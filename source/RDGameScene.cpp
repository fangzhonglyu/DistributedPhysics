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
#include "RDGameScene.h"
#include <box2d/b2_world.h>
#include <box2d/b2_contact.h>
#include <box2d/b2_collision.h>

#include <ctime>
#include <string>
#include <iostream>
#include <sstream>

using namespace cugl;


#pragma mark -
#pragma mark Level Geography

/** This is the size of the active portion of the screen */
#define SCENE_WIDTH 1024
#define SCENE_HEIGHT 576

/** Width of the game world in Box2d units */
#define DEFAULT_WIDTH   32.0f
/** Height of the game world in Box2d units */
#define DEFAULT_HEIGHT  18.0f
/** The default value of gravity (going down) */
#define DEFAULT_GRAVITY -4.9f

#define DEFAULT_TURN_RATE 0.05f

/** To automate the loading of crate files */
#define NUM_CRATES 15

#define INPUT_DELAY 2

#define LOG_MSGS 0

// Since these appear only once, we do not care about the magic numbers.
// In an actual game, this information would go in a data file.
// IMPORTANT: Note that Box2D units do not equal drawing units
/** The wall vertices */
float WALL1[] = { 0.0f,  0.0f, 16.0f,  0.0f, 16.0f,  1.0f,
                  3.0f,  1.0f,  3.0f,  5.0f,  2.0f,  7.0f,
                  1.0f, 17.0f,  8.0f, 15.0f, 16.0f, 17.0f,
                 16.0f, 18.0f,  0.0f, 18.0f};
float WALL2[] = {32.0f, 18.0f, 16.0f, 18.0f, 16.0f, 17.0f,
                 31.0f, 16.0f, 30.0f, 10.0f, 31.0f,  1.0f,
                 16.0f,  1.0f, 16.0f,  0.0f, 32.0f,  0.0f};
float WALL3[] = { 4.0f,  9.5f,  8.0f,  9.5f,
                  8.0f, 10.5f,  4.0f, 10.5f };

/** The positions of the crate pyramid */
float BOXES[] = { 14.5f, 14.25f,
                  13.0f, 12.00f, 16.0f, 12.00f,
                  11.5f,  9.75f, 14.5f,  9.75f, 17.5f, 9.75f,
                  13.0f,  7.50f, 16.0f,  7.50f,
                  11.5f,  5.25f, 14.5f,  5.25f, 17.5f, 5.25f,
                  10.0f,  3.00f, 13.0f,  3.00f, 16.0f, 3.00f, 19.0f, 3.0f};

/** The initial cannon position */
float CAN1_POS[] = { 2, 9 };
float CAN2_POS[] = { 30,9 };
/** The goal door position */
float GOAL_POS[] = { 6, 12};

#pragma mark Assset Constants
/** The key for the earth texture in the asset manager */
#define EARTH_TEXTURE       "earth"
/** The key for the rocket texture in the asset manager */
#define ROCK_TEXTURE        "rocket"
/** The key for the win door texture in the asset manager */
#define GOAL_TEXTURE        "goal"
/** The key prefix for the multiple crate assets */
#define CRATE_PREFIX        "crate"
/** The key for the fire textures in the asset manager */
#define MAIN_FIRE_TEXTURE   "flames"
#define RGHT_FIRE_TEXTURE   "flames-right"
#define LEFT_FIRE_TEXTURE   "flames-left"

/** Color to outline the physics nodes */
#define STATIC_COLOR    Color4::WHITE
/** Opacity of the physics outlines */
#define DYNAMIC_COLOR   Color4::YELLOW

/** The key for collisions sounds */
#define COLLISION_SOUND     "bump"
/** The key for the main afterburner sound */
#define MAIN_FIRE_SOUND     "burn"
/** The key for the right afterburner sound */
#define RGHT_FIRE_SOUND     "right-burn"
/** The key for the left afterburner sound */
#define LEFT_FIRE_SOUND     "left-burn"

/** The key for the font reference */
#define PRIMARY_FONT        "retro"

#pragma mark Physics Constants

// Physics constants for initialization
/** Density of non-crate objects */
#define BASIC_DENSITY       0.0f
/** Density of the crate objects */
#define CRATE_DENSITY       1.0f
/** Friction of non-crate objects */
#define BASIC_FRICTION      0.1f
/** Friction of the crate objects */
#define CRATE_FRICTION      0.2f
/** Angular damping of the crate objects */
#define CRATE_DAMPING       1.0f
/** Collision restitution for all objects */
#define BASIC_RESTITUTION   0.1f
/** Threshold for generating sound on collision */
#define SOUND_THRESHOLD     3


#pragma mark -
#pragma mark Constructors
/**
 * Creates a new game world with the default values.
 *
 * This constructor does not allocate any objects or start the controller.
 * This allows us to use a controller without a heap pointer.
 */
GameScene::GameScene() : cugl::Scene2(),
_complete(false),
_debug(false),
_isHost(false)
{    
}

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
bool GameScene::init(const std::shared_ptr<AssetManager>& assets) {
    return init(assets,Rect(0,0,DEFAULT_WIDTH,DEFAULT_HEIGHT),Vec2(0,DEFAULT_GRAVITY));
}

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
bool GameScene::init(const std::shared_ptr<AssetManager>& assets, const Rect rect) {
    return init(assets,rect,Vec2(0,DEFAULT_GRAVITY));
}

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
bool GameScene::init(const std::shared_ptr<AssetManager>& assets, const Rect rect, const Vec2 gravity) {
    Size dimen = computeActiveSize();

    if (assets == nullptr) {
        return false;
    } else if (!Scene2::init(dimen)) {
        return false;
    }
    
    // Start up the input handler
    _assets = assets;
    _input.init();
    _input.update(0);
    _rand.seed(0xdeadbeef);

    // IMPORTANT: SCALING MUST BE UNIFORM
    // This means that we cannot change the aspect ratio of the physics world
    // Shift to center if a bad fit
    _scale = dimen.width == SCENE_WIDTH ? dimen.width/rect.size.width : dimen.height/rect.size.height;
    Vec2 offset((dimen.width-SCENE_WIDTH)/2.0f,(dimen.height-SCENE_HEIGHT)/2.0f);

    // Create the scene graph
    _worldnode = scene2::SceneNode::alloc();
    _worldnode->setAnchor(Vec2::ANCHOR_BOTTOM_LEFT);
    _worldnode->setPosition(offset);
    
    _debugnode = scene2::SceneNode::alloc();
    _debugnode->setScale(_scale); // Debug node draws in PHYSICS coordinates
    _debugnode->setAnchor(Vec2::ANCHOR_BOTTOM_LEFT);
    _debugnode->setPosition(offset);

    _winnode = scene2::Label::allocWithText("VICTORY!",_assets->get<Font>(PRIMARY_FONT));
	_winnode->setAnchor(Vec2::ANCHOR_CENTER);
    _winnode->setPosition(dimen/2.0f);
    _winnode->setForeground(STATIC_COLOR);
    _winnode->setVisible(false);
    
    _chargeBar = std::dynamic_pointer_cast<scene2::ProgressBar>(assets->get<scene2::SceneNode>("load_bar"));
    _chargeBar->setPosition(Vec2(dimen.width/2.0f,dimen.height*0.9f));
    
    addChild(_worldnode);
    addChild(_debugnode);
    addChild(_winnode);
    addChild(_chargeBar);
    
    _world = physics2::ObstacleWorld::alloc(Rect(0,0,DEFAULT_WIDTH,DEFAULT_HEIGHT),Vec2(0,DEFAULT_GRAVITY));
    _world->update(FIXED_TIMESTEP_S);
    
    populate(true);
    _active = true;
    _complete = false;
    setDebug(false);
    
    _netCache.clear();
    _serializer.reset();
    _deserializer.reset();
    
    _counter = 0;
    
    // XNA nostalgia
    Application::get()->setClearColor(Color4f::CORNFLOWER);
    return true;
}

/**
 * Disposes of all (non-static) resources allocated to this mode.
 */
void GameScene::dispose() {
    if (_active) {
        removeAllChildren();
        _input.dispose();
        _world = nullptr;
        _worldnode = nullptr;
        _debugnode = nullptr;
        _winnode = nullptr;
        _complete = false;
        _debug = false;
        if(LOG_MSGS){
            _writer->flush();
            _writer->close();
        }
        Scene2::dispose();
    }
}

void GameScene::setHost(bool isHost){
    _isHost = isHost;
    if(LOG_MSGS){
        _writer = _writer->alloc(isHost?"log_host.txt":"log_client.txt");
    }
}

#pragma mark -
#pragma mark Level Layout

std::shared_ptr<physics2::PolygonObstacle> wallobj1;
std::shared_ptr<physics2::PolygonObstacle> wallobj2;
std::shared_ptr<scene2::PolygonNode> wallsprite1;
std::shared_ptr<scene2::PolygonNode> wallsprite2;

std::vector<std::shared_ptr<physics2::BoxObstacle>> boxes;
std::vector<std::shared_ptr<scene2::PolygonNode>> nodes;

/**
 * Resets the status of the game so that we can play again.
 *
 * This method disposes of the world and creates a new one.
 */
void GameScene::reset() {
    CULog("reset");
    if(LOG_MSGS){
        _writer->writeLine("reset");
    }
    _rand.seed(0xdeadbeef);
    _netCache.clear();
    _worldnode->removeAllChildren();
    _debugnode->removeAllChildren();
    setComplete(false);
    populate(false);
    Application::get()->resetLeftOver();
}

std::shared_ptr<physics2::BoxObstacle> GameScene::addCrateAt(cugl::Vec2 pos, bool original) {
    // Pick a crate and random and generate the key
    int indx = (_rand() % 2 == 0 ? 2 : 1);
    std::stringstream ss;
    ss << CRATE_PREFIX << (indx < 10 ? "0" : "" ) << indx;

    // Create the sprite for this crate
    auto image  = _assets->get<Texture>(ss.str());

    Size boxSize(image->getSize()/_scale);
    auto crate = physics2::BoxObstacle::alloc(pos,boxSize);
    crate->setDebugColor(DYNAMIC_COLOR);
    crate->setName(ss.str());
    crate->setAngleSnap(0); // Snap to the nearest degree

    // Set the physics attributes
    crate->setDensity(CRATE_DENSITY);
    crate->setFriction(CRATE_FRICTION);
    crate->setAngularDamping(CRATE_DAMPING);
    crate->setRestitution(BASIC_RESTITUTION);
    
    auto sprite = scene2::PolygonNode::allocWithTexture(image);
    sprite->setAnchor(Vec2::ANCHOR_CENTER);
    
    if(original){
        boxes.push_back(crate);
        nodes.push_back(sprite);
    }
    
    addObstacle(crate,sprite);   // PUT SAME TEXTURES IN SAME LAYER!!!
    return crate;
}


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
void GameScene::populate(bool isInit) {
    
    // have to completely reset the world
    Timestamp start;
    
    _nextObj = 0;
    _objQueue = std::queue<Uint32>();
    _objMap.clear();
    _itpr.reset();
    
    _world = physics2::ObstacleWorld::alloc(Rect(0,0,DEFAULT_WIDTH,DEFAULT_HEIGHT),Vec2(0,DEFAULT_GRAVITY));
    _world->activateCollisionCallbacks(true);
    _world->onBeginContact = [this](b2Contact* contact) {
        beginContact(contact);
    };
    _world->beforeSolve = [this](b2Contact* contact, const b2Manifold* oldManifold) {
        beforeSolve(contact,oldManifold);
    };
    
    Timestamp world;
    CULog("World reinit in %fms",world.ellapsedMicros(start)/1000.f);
    
    if(isInit){
        std::shared_ptr<Texture> image;
        //std::shared_ptr<scene2::PolygonNode> sprite;
        
#pragma mark : Wall polygon 1
        
        // Create ground pieces
        // All walls share the same texture
        image  = _assets->get<Texture>(EARTH_TEXTURE);
        std::string wname = "wall";

        // Create the polygon outline
        Poly2 wall1(reinterpret_cast<Vec2*>(WALL1),11);
        EarclipTriangulator triangulator;
        triangulator.set(wall1.vertices);
        triangulator.calculate();
        wall1.setIndices(triangulator.getTriangulation());
        triangulator.clear();

        //std::shared_ptr<physics2::PolygonObstacle> wallobj;
        wallobj1 = physics2::PolygonObstacle::allocWithAnchor(wall1,Vec2::ANCHOR_CENTER);
        wallobj1->setDebugColor(STATIC_COLOR);
        wallobj1->setName(wname);

        // Set the physics attributes
        wallobj1->setBodyType(b2_staticBody);
        wallobj1->setDensity(BASIC_DENSITY);
        wallobj1->setFriction(BASIC_FRICTION);
        wallobj1->setRestitution(BASIC_RESTITUTION);

        // Add the scene graph nodes to this object
        wall1 *= _scale;
        wallsprite1 = scene2::PolygonNode::allocWithTexture(image,wall1);
        
#pragma mark : Wall polygon 2
        Poly2 wall2(reinterpret_cast<Vec2*>(WALL2),9);
        triangulator.set(wall2.vertices);
        triangulator.calculate();
        wall2.setIndices(triangulator.getTriangulation());
        triangulator.clear();

        wallobj2 = physics2::PolygonObstacle::allocWithAnchor(wall2,Vec2::ANCHOR_CENTER);
        wallobj2->setDebugColor(STATIC_COLOR);
        wallobj2->setName(wname);

        // Set the physics attributes
        wallobj2->setBodyType(b2_staticBody);
        wallobj2->setDensity(BASIC_DENSITY);
        wallobj2->setFriction(BASIC_FRICTION);
        wallobj2->setRestitution(BASIC_RESTITUTION);

        // Add the scene graph nodes to this object
        wall2 *= _scale;
        wallsprite2 = scene2::PolygonNode::allocWithTexture(image,wall2);
        
#pragma mark : Crates
        for (int ii = 0; ii < NUM_CRATES; ii++) {
            // Pick a crate and random and generate the key
            Vec2 boxPos(BOXES[2*ii], BOXES[2*ii+1]);
            addCrateAt(boxPos,true);
        }
        
#pragma mark : Cannon
        image  = _assets->get<Texture>(ROCK_TEXTURE);
        
        _cannon1Node = scene2::PolygonNode::allocWithTexture(image);
        
        Size canSize(image->getSize()/_scale);
        
        Vec2 canPos1 = ((Vec2)CAN1_POS);
        _cannon1 = CannonModel::alloc(canPos1,canSize,DEFAULT_TURN_RATE);
        _cannon1->setBodyType(b2BodyType::b2_kinematicBody);
        _cannon1->setDrawScale(_scale);
        _cannon1->setAngle(-M_PI_2);
        _cannon1->setDebugColor(DYNAMIC_COLOR);
        _cannon1->setSensor(true);
        _cannon1->setCannonNode(_cannon1Node);
            
        image  = _assets->get<Texture>(ROCK_TEXTURE);
        _cannon2Node = scene2::PolygonNode::allocWithTexture(image);
        
        Vec2 canPos2 = ((Vec2)CAN2_POS);
        _cannon2= CannonModel::alloc(canPos2,canSize,-DEFAULT_TURN_RATE);
        _cannon2->setBodyType(b2BodyType::b2_kinematicBody);
        _cannon2->setDrawScale(_scale);
        _cannon2->setAngle(M_PI_2);
        _cannon2->setDebugColor(DYNAMIC_COLOR);
        _cannon2->setSensor(true);
        _cannon2->setCannonNode(_cannon2Node);
    }
    
    if(isInit){
        addObstacle(wallobj1,wallsprite1);  // All walls share the same texture
        addObstacle(wallobj2,wallsprite2);  // All walls share the same texture
    }
    else{
        addObstacleAlt(wallobj1,wallsprite1);  // All walls share the same texture
        addObstacleAlt(wallobj2,wallsprite2);  // All walls share the same texture
        for(int ii = 0; ii < NUM_CRATES; ii++){
            Vec2 boxPos(BOXES[2*ii], BOXES[2*ii+1]);
            boxes[ii]->setPosition(boxPos);
            boxes[ii]->setLinearVelocity(Vec2::ZERO);
            boxes[ii]->setAngle(0);
            boxes[ii]->setAngularVelocity(0);
            addObstacleAlt(boxes[ii], nodes[ii]);
        }
    }
    
    Vec2 canPos1 = ((Vec2)CAN1_POS);
    _cannon1->setPosition(canPos1);
    _cannon1->setAngle(-M_PI_2);
    _world->addObstacle(_cannon1);
    _worldnode->addChild(_cannon1Node);
    
    Vec2 canPos2 = ((Vec2)CAN2_POS);
    _cannon2->setPosition(canPos2);
    _cannon2->setAngle(M_PI_2);
    _world->addObstacle(_cannon2);
    _worldnode->addChild(_cannon2Node);

    Timestamp end;
    CULog("World reset in %fms",end.ellapsedMicros(start)/1000.f);
}



/**
 * Adds the physics object to the physics world and loosely couples it to the scene graph
 *
 * There are two ways to link a physics object to a scene graph node on the
 * screen.  One way is to make a subclass of a physics object, like we did 
 * with rocket.  The other is to use callback functions to loosely couple 
 * the two.  This function is an example of the latter.
 *
 * param obj    The physics object to add
 * param node   The scene graph node to attach it to
 */
void GameScene::addObstacle(const std::shared_ptr<physics2::Obstacle>& obj,
                            const std::shared_ptr<scene2::SceneNode>& node) {
    _world->addObstacle(obj);
    
    _objQueue.push(_nextObj);
    _objMap.insert(std::make_pair(_nextObj,obj));
    _nextObj++;
    
    //obj->setDebugScene(_debugnode);
    
    // Position the scene graph node (enough for static objects)
    node->setPosition(obj->getPosition()*_scale);
    _worldnode->addChild(node);
    
    // Dynamic objects need constant updating
    if (obj->getBodyType() == b2_dynamicBody) {
        scene2::SceneNode* weak = node.get(); // No need for smart pointer in callback
        obj->setListener([=](physics2::Obstacle* obs){
            float leftover = Application::get()->getLeftOver()/1000000.f;
            Vec2 pos = obs->getPosition()+leftover*obs->getLinearVelocity();
            float angle = obs->getAngle()+leftover*obs->getAngularVelocity();
            weak->setPosition(pos*_scale);
            weak->setAngle(angle);
        });
    }
}

void GameScene::addObstacleAlt(const std::shared_ptr<cugl::physics2::Obstacle>& obj,
                    const std::shared_ptr<cugl::scene2::SceneNode>& node){
    _world->addObstacle(obj);
    
    _objQueue.push(_nextObj);
    _objMap.insert(std::make_pair(_nextObj,obj));
    _nextObj++;
    
    node->setPosition(obj->getPosition()*_scale);
    _worldnode->addChild(node);
}

netdata GameScene::packFire(Uint64 timestamp){
    netdata data;
    data.timestamp = timestamp;
    data.flag = FIRE_INPUT_FLAG;
    _serializer.reset();
    _serializer.writeBool(_isHost);
    auto cannon = _isHost ? _cannon1Node : _cannon2Node;
    float angle = cannon->getAngle();
    _serializer.writeFloat(angle);
    float firepower = _input.getFirePower();
    _serializer.writeFloat(firepower);
    data.data = _serializer.serialize();
    //float delayMs = (timestamp-_counter)*FIXED_TIMESTEP_S*1000;
    //CULog("Fire input at angle %f, power %f, cached for %llu, added delay of %.2fms",angle,firepower,timestamp,delayMs);
    return data;
}

netdata GameScene::packState(Uint64 timestamp){
    Uint32 id = _objQueue.front();
    _objQueue.pop();
    auto obj = _objMap.at(id);
    netdata data;
    data.timestamp = timestamp;
    data.flag = STATE_SYNC_FLAG;
    _serializer.reset();
    _serializer.writeUint32(id);
    _serializer.writeFloat(obj->getX());
    _serializer.writeFloat(obj->getY());
    _serializer.writeFloat(obj->getVX());
    _serializer.writeFloat(obj->getVY());
    _serializer.writeFloat(obj->getAngle());
    _serializer.writeFloat(obj->getAngularVelocity());
    data.data = _serializer.serialize();
    _objQueue.push(id);
    return data;
}

netdata GameScene::packReset(Uint64 timestamp){
    netdata data;
    data.timestamp = timestamp;
    data.flag = RESET_FLAG;
    data.data = std::vector<std::byte>();
    return data;
}

netdata GameScene::packCannon(Uint64 timestamp){
    netdata data;
    data.timestamp = timestamp;
    data.flag = CANNON_FLAG;
    _serializer.reset();
    _serializer.writeBool(_isHost);
    auto cannon = _isHost? _cannon1: _cannon2;
    _serializer.writeFloat(cannon->getAngle());
    data.data = _serializer.serialize();
    return data;
}

union {
    float f;
    uint32_t u;
} f2u;

void GameScene::processFire(netdata data){
    CUAssert(data.flag == FIRE_INPUT_FLAG);
    if(data.timestamp < _counter){
        if(LOG_MSGS){
            _writer->writeLine("Simulation Corrupted, State Synchronization Needed");
        }
        CULogError("Simulation Corrupted, State Synchronization Needed");
    }
    if(data.timestamp <= _counter){
        _deserializer.reset();
        _deserializer.reset();
        _deserializer.receive(data.data);
        bool isHost = _deserializer.readBool();
        auto cannon = isHost ? _cannon1Node : _cannon2Node;
        float angle = _deserializer.readFloat() + M_PI_2;
        float firePower = _deserializer.readFloat();
        Vec2 forward(SDL_cosf(angle),SDL_sinf(angle));
        Vec2 pos = cannon->getPosition()/_scale+1.5f*forward;
        auto crate = addCrateAt(pos,false);
        //crate->setBodyType(b2_kinematicBody);
        crate->setLinearVelocity(forward*50*firePower);
    
        f2u.f = firePower;
        uint32_t fpu = f2u.u;
        f2u.f = angle;
        f2u.f = angle;
        uint32_t au = f2u.u;
        if(LOG_MSGS){
            _writer->writeLine(cugl::strtool::format("Cannon %d fire, angle: %u, power: %u",isHost ? 1 : 2,au,fpu));
        }
        CULog("Cannon %d fire, angle: %u, power: %u",isHost ? 1 : 2,au,fpu);
    }
}

void GameScene::processState(netdata data){
    if(data.sourceID == ""){
        //CULog("Ignoring state sync from self.");
        return;
    }
    CUAssert(data.flag == STATE_SYNC_FLAG);
    if(data.timestamp < _counter){
        if(LOG_MSGS){
            _writer->writeLine("Outdated state, extrapolating");
        }
        //CULog("Outdated state, extrapolating");
    }
    _deserializer.reset();
    _deserializer.receive(data.data);
    Uint32 id = _deserializer.readUint32();
    float x = _deserializer.readFloat();
    float y = _deserializer.readFloat();
    float vx = _deserializer.readFloat();
    float vy = _deserializer.readFloat();
    float angle = _deserializer.readFloat();
    float angV = _deserializer.readFloat();
    //CULog("state sync for obj %u, %f,%f,|%f,%f|%f,%f",id,x,y,vx,vy,angle,angV);
    if(!_objMap.count(id)){
        CULogError("unknown object");
        return;
    }
    auto obj = _objMap.at(id);
    float diff = (obj->getPosition()-Vec2(x,y)).length();
    float angDiff = 10*abs(obj->getAngle()-angle);
    int steps = SDL_max(0,SDL_min(30,SDL_max((int)(diff*30),(int)angDiff)));
    //x+=(steps)*FIXED_TIMESTEP_S*vx;
    //y+=(steps)*FIXED_TIMESTEP_S*vy;
    
    std::vector<float> param = std::vector<float>();
    param.push_back(x); param.push_back(y); param.push_back(vx); param.push_back(vy); param.push_back(angle); param.push_back(angV);
    
    _itpr.addObject(obj, std::make_pair(steps,param));
//    obj->setPosition(x, y);
//    obj->setLinearVelocity(vx, vy);
//    obj->setAngle(angle);
//    obj->setAngularVelocity(angV);
//    obj->setPosition(param[0], param[1]);
//    obj->setLinearVelocity(param[2], param[3]);
//    obj->setAngle(param[4]);
//    obj->setAngularVelocity(param[5]);
}

void GameScene::processCannon(netdata data){
    if(data.sourceID == ""){
        //CULog("Ignoring state sync from self.");
        return;
    }
    CUAssert(data.flag == CANNON_FLAG);
    _deserializer.reset();
    _deserializer.receive(data.data);
    auto cannon = _deserializer.readBool() ? _cannon1:_cannon2;
    float angle = _deserializer.readFloat();
    float angDiff = 10*abs(cannon->getAngle()-angle);
    int steps = SDL_max(0,SDL_min(30,(int)angDiff));
    
    std::vector<float> param = std::vector<float>();
    param.push_back(cannon->getX()); param.push_back(cannon->getY()); param.push_back(cannon->getVX()); param.push_back(cannon->getVY()); param.push_back(angle); param.push_back(0);
    
    _itpr.addObject(cannon, std::make_pair(steps,param));
}

void GameScene::processCache(){
    
    if(_netCache.isEmpty()){
        return;
    }
    while(!_netCache.isEmpty() && _netCache.peek().timestamp <= _counter){
        netdata next = _netCache.pop();
        if(LOG_MSGS){
            _writer->writeLine(cugl::strtool::format("MESSAGE at %llu, received by %llu", next.timestamp, next.receivedBy));
        }
        if(next.flag != STATE_SYNC_FLAG){
            CULog("MESSAGE at %llu, received by %llu", next.timestamp, next.receivedBy);
        }
        switch (next.flag) {
            case FIRE_INPUT_FLAG:
                processFire(next);
                break;
            case RESET_FLAG:
                reset();
                break;
            case STATE_SYNC_FLAG:
                processState(next);
                break;
            case CANNON_FLAG:
                processCannon(next);
                break;
            default:
                CULogError("unknown flag %u", next.flag);
                break;
        }
    }
}

bool GameScene::checkConnection() {
    auto state = _network->getState();
    if (state == cugl::net::NetcodeConnection::State::CONNECTED) {
        return true;
    }
    else {
        _network = nullptr;
        _complete = true;
        return false;
    }
}

void GameScene::updateNet(){
    if (_network) {
        _network->receive([this](const std::string source,
                                 const std::vector<std::byte>& data) {
            processData(source, data);
        });
        checkConnection();
    }
}

void GameScene::transmitNetdata(const netdata data){
    _serializer.reset();
    _serializer.writeUint64(data.timestamp);
    _serializer.writeUint32(data.flag);
    auto arr = _serializer.serialize();
    arr.insert(std::end(arr), std::begin(data.data), std::end(data.data));
    //_netCache.push(data, _counter);
    _network->broadcast(arr);
}

void GameScene::processData(const std::string source,
                            const std::vector<std::byte>& data) {
//    CULog(source.c_str());
//    CULog("gotdata");
    _deserializer.reset();
    _deserializer.receive(data);
    netdata msg;
    msg.timestamp = _deserializer.readUint64();
    msg.sourceID = source;
    msg.flag = _deserializer.readUint32();
    msg.data = { data.begin() + 14, data.end()};
    msg.receivedBy = _counter;
    _netCache.push(msg, _counter);
}

#pragma mark -
#pragma mark Physics Handling

#if USING_PHYSICS
void GameScene::preUpdate(float dt) {
    _input.update(dt);
    
    if(_input.getFirePower()>0.f){
        _chargeBar->setVisible(true);
        _chargeBar->setProgress(_input.getFirePower());
    }
    else{
        _chargeBar->setVisible(false);
    }

    // Process the toggled key commands
    if (_input.didDebug()) { setDebug(!isDebug()); }
    if (_input.didReset()) {
        transmitNetdata(packReset(_counter+INPUT_DELAY));
    }
    if (_input.didExit()) {
        CULog("Shutting down");
        Application::get()->quit();
    }
    
    if (_input.didFire()) {
        transmitNetdata(packFire(_counter+INPUT_DELAY));
    }
    
    if (!_objQueue.empty() && _isHost){
        for(int ii = 0; ii < 5; ii++){
            transmitNetdata(packState(_counter));
        }
    }
    
    float turnRate = _isHost ? DEFAULT_TURN_RATE : -DEFAULT_TURN_RATE;
    auto cannon = _isHost ? _cannon1 : _cannon2;
    //cannon->setAngularVelocity(_input.getVertical() * turnRate);
    cannon->setAngle(_input.getVertical() * turnRate + cannon->getAngle());
    
    transmitNetdata(packCannon(_counter));
}

void GameScene::postUpdate(float dt) {
    //Nothing to do now
}

void GameScene::fixedUpdate() {
    updateNet();
    processCache();
    _world->update(FIXED_TIMESTEP_S);
    _itpr.fixedUpdate();
    _counter++;
}

#else
/**
 * Executes the core gameplay loop of this world.
 *
 * This method contains the specific update code for this mini-game. It does
 * not handle collisions, as those are managed by the parent class WorldController.
 * This method is called after input is read, but before collisions are resolved.
 * The very last thing that it should do is apply forces to the appropriate objects.
 *
 * @param  delta    Number of seconds since last animation frame
 */
void GameScene::update(float dt) {
    return;
//    preUpdate(dt);
//    _world->update(dt);
//    postUpdate(dt);
}
#endif

/**
 * Processes the start of a collision
 *
 * This method is called when we first get a collision between two objects.  We use
 * this method to test if it is the "right" kind of collision.  In particular, we
 * use it to test if we make it to the win door.
 *
 * @param  contact  The two bodies that collided
 */
void GameScene::beginContact(b2Contact* contact) {}

/**
 * Handles any modifications necessary before collision resolution
 *
 * This method is called just before Box2D resolves a collision.  We use this method
 * to implement sound on contact, using the algorithms outlined in Ian Parberry's
 * "Introduction to Game Physics with Box2D".
 *
 * @param  contact  	The two bodies that collided
 * @param  oldManfold  	The collision manifold before contact
 */
void GameScene::beforeSolve(b2Contact* contact, const b2Manifold* oldManifold) {
    float speed = 0;

    // Use Ian Parberry's method to compute a speed threshold
    b2Body* body1 = contact->GetFixtureA()->GetBody();
    b2Body* body2 = contact->GetFixtureB()->GetBody();
    b2WorldManifold worldManifold;
    contact->GetWorldManifold(&worldManifold);
    b2PointState state1[2], state2[2];
    b2GetPointStates(state1, state2, oldManifold, contact->GetManifold());
    for(int ii =0; ii < 2; ii++) {
        if (state2[ii] == b2_addState) {
            b2Vec2 wp = worldManifold.points[0];
            b2Vec2 v1 = body1->GetLinearVelocityFromWorldPoint(wp);
            b2Vec2 v2 = body2->GetLinearVelocityFromWorldPoint(wp);
            b2Vec2 dv = v1-v2;
            speed = b2Dot(dv,worldManifold.normal);
        }
    }
    
    // Play a sound if above threshold
    if (speed > SOUND_THRESHOLD) {
        // These keys result in a low number of sounds.  Too many == distortion.
        physics2::Obstacle* data1 = reinterpret_cast<physics2::Obstacle*>(body1->GetUserData().pointer);
        physics2::Obstacle* data2 = reinterpret_cast<physics2::Obstacle*>(body2->GetUserData().pointer);

        if (data1 != nullptr && data2 != nullptr) {
            std::string key = (data1->getName()+data2->getName());
            auto source = _assets->get<Sound>(COLLISION_SOUND);
            if (!AudioEngine::get()->isActive(key)) {
                AudioEngine::get()->play(key, source, false, source->getVolume());
            }
        }
    }
}

/**
 * Returns the active screen size of this scene.
 *
 * This method is for graceful handling of different aspect
 * ratios
 */
Size GameScene::computeActiveSize() const {
    Size dimen = Application::get()->getDisplaySize();
    float ratio1 = dimen.width/dimen.height;
    float ratio2 = ((float)SCENE_WIDTH)/((float)SCENE_HEIGHT);
    if (ratio1 < ratio2) {
        dimen *= SCENE_WIDTH/dimen.width;
    } else {
        dimen *= SCENE_HEIGHT/dimen.height;
    }
    return dimen;
}
