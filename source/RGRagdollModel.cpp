//
//  RGRagdollModel.h
//  Ragdoll Demo
//
//  This module provides the infamous Walker White ragdoll from 3152.  This time it
//  is fully assembled for you.
//
//  WARNING: There are a lot of shortcuts in this design that will do not adapt
//  well to data driven design.  This demo has a lot of simplifications to make
//  it a bit easier to see how everything fits together.  However, the model
//  classes and how they are initialized will need to be changed if you add
//  dynamic level loading.
//
//  Pay close attention to how this class designed. This class uses our standard
//  shared-pointer architecture which is common to the entire engine.
//
//  1. The constructor does not perform any initialization; it just sets all
//     attributes to their defaults.
//
//  2. All initialization takes place via init methods, which can fail if an
//     object is initialized more than once.
//
//  3. All allocation takes place via static constructors which return a shared
//     pointer.
//
//  This file is based on the CS 3152 PhysicsDemo Lab by Don Holden, 2007
//
//  Author: Walker White and Anthony Perello
//  Version: 1/26/17
//
#include "RGRagdollModel.h"
#include <box2d/b2_revolute_joint.h>
#include <box2d/b2_weld_joint.h>
#include <box2d/b2_world.h>


using namespace cugl;

#pragma mark -
#pragma mark Animation and Physics Constants

/** This is adjusted by screen aspect ratio to get the height */
#define GAME_WIDTH 1024

/** The offset of the snorkel from the doll's head (in Box2d units) */
float BUBB_OFF[] = { 0.55f,  1.9f };

#pragma mark -
#pragma mark Constructors

/**
 * Initializes a new Ragdoll with the given position
 *
 * The Ragdoll is 1 unit by 1 unit in size. The Ragdoll is scaled so that
 * 1 pixel = 1 Box2d unit
 *
 * The scene graph is completely decoupled from the physics system.
 * The node does not have to be the same size as the physics body. We
 * only guarantee that the scene graph node is positioned correctly
 * according to the drawing scale.
 *
 * @param  pos		Initial position in world coordinates
 * @param  scale    The drawing scale to convert between world and screen coordinates
 *
 * @return  true if the obstacle is initialized properly, false otherwise.
 */
bool RagdollModel::init(float scale) {
	_drawscale = scale;
	return true;
}

/**
 * Disposes all resources and assets of this Ragdoll
 *
 * Any assets owned by this object will be immediately released.  Once
 * disposed, a Ragdoll may not be used until it is initialized again.
 */
void RagdollModel::dispose() {
    _node = nullptr;
    _bodies.clear();
	_joints.clear();
}


#pragma mark -
#pragma mark Part Initialization
/**
 * Creates the individual body parts for this ragdoll
 *
 * The size of the body parts is determined by the scale together with
 * the assets (as part of the asset manager).  This will fail if any
 * body part assets are missing.
 *
 * @param assets The program asset manager
 *
 * @return true if the body parts were successfully created
 */
bool RagdollModel::buildParts(const std::shared_ptr<AssetManager>& assets, Vec2 pos) {
    CUAssertLog(_bodies.empty(), "Bodies are already initialized");
   
    // Get the images from the asset manager
    bool success = true;
    for(int ii = 0; ii <= PART_RIGHT_SHIN; ii++) {
        std::string name = getPartName(ii);
        std::shared_ptr<Texture> image = assets->get<Texture>(name);
        if (image == nullptr) {
            success = false;
        } else {
            _textures.push_back(image);
        }
    }
    if (!success) {
        return false;
    }
    
    // Now make everything
    std::shared_ptr<physics2::BoxObstacle> part;
    
    // TORSO;
    part = makePart(PART_BODY, PART_NONE, pos);
    part->setFixedRotation(true);

	_base = part;
    
    // HEAD
    makePart(PART_HEAD, PART_BODY, Vec2(0, TORSO_OFFSET));
    
    // ARMS
    makePart(PART_LEFT_ARM, PART_BODY, Vec2(-ARM_XOFFSET, ARM_YOFFSET));
    makePart(PART_RIGHT_ARM, PART_BODY, Vec2(ARM_XOFFSET, ARM_YOFFSET));
    
    // FOREARMS
    makePart(PART_LEFT_FOREARM, PART_LEFT_ARM, Vec2(-FOREARM_OFFSET, 0));
    makePart(PART_RIGHT_FOREARM, PART_RIGHT_ARM, Vec2(FOREARM_OFFSET, 0));
    
    // THIGHS
    makePart(PART_LEFT_THIGH, PART_BODY, Vec2(-THIGH_XOFFSET, -THIGH_YOFFSET));
    makePart(PART_RIGHT_THIGH, PART_BODY, Vec2(THIGH_XOFFSET, -THIGH_YOFFSET));
    
    // SHINS
    makePart(PART_LEFT_SHIN, PART_LEFT_THIGH, Vec2(0, -SHIN_OFFSET));
    makePart(PART_RIGHT_SHIN, PART_RIGHT_THIGH, Vec2(0, -SHIN_OFFSET));
    return true;
}

/**
 * Sets the texture for the given body part.
 *
 * As some body parts are symmetrical, we may reuse textures.
 *
 * @param part      The part identifier
 * @param texture   The texture for the given body part
 */
void RagdollModel::setPart(int part, const std::shared_ptr<Texture>& texture) {
    if (_textures.size() <= PART_RIGHT_SHIN) {
        _textures.resize(PART_RIGHT_SHIN+1, nullptr);
    }
    _textures[part] = texture;
}


/**
* Returns a single body part
*
* While it looks like this method "connects" the pieces, it does not really.
* It puts them in position to be connected by joints, but they will fall apart
* unless you make the joints.
*
* @param  part     Part to create
* @param  connect  Part to connect it to
* @param  pos      Position RELATIVE to connecting part
*
* @return the created body part
*/
std::shared_ptr<physics2::BoxObstacle> RagdollModel::makePart(int part, int connect, const Vec2& pos) {
	std::shared_ptr<Texture> image = _textures[part];
	Size size = image->getSize();
	size.width /= _drawscale;
	size.height /= _drawscale;

	Vec2 pos2 = pos;
	if (connect != PART_NONE) {
		pos2 += _bodies[connect]->getPosition();
	}

	std::shared_ptr<physics2::BoxObstacle> body = physics2::BoxObstacle::alloc(pos2, size);
	body->setName(getPartName(part));
	body->setDensity(DEFAULT_DENSITY);
	
	_bodies.push_back(body);
	return body;
}

/**
 * Returns the texture key for the given body part.
 *
 * As some body parts are symmetrical, we reuse textures.
 *
 * @return the texture key for the given body part
 */
std::string RagdollModel::getPartName(int part) {
	switch (part) {
	case PART_HEAD:
		return HEAD_TEXTURE;
	case PART_BODY:
		return BODY_TEXTURE;
	case PART_LEFT_ARM:
	case PART_RIGHT_ARM:
		return ARM_TEXTURE;
	case PART_LEFT_FOREARM:
	case PART_RIGHT_FOREARM:
		return FOREARM_TEXTURE;
	case PART_LEFT_THIGH:
	case PART_RIGHT_THIGH:
		return THIGH_TEXTURE;
	case PART_LEFT_SHIN:
	case PART_RIGHT_SHIN:
		return SHIN_TEXTURE;
	default:
		return "UNKNOWN";
	}
}

#pragma mark -
#pragma mark Physics
/**
 * Updates the object's physics state (NOT GAME LOGIC).
 *
 * This method is called AFTER the collision resolution state. Therefore, it
 * should not be used to process actions or any other gameplay information.
 * Its primary purpose is to adjust changes to the fixture, which have to
 * take place after collision.
 *
 * In other words, this is the method that updates the scene graph.  If you
 * forget to call it, it will not draw your changes.
 *
 * @param delta Timing values from parent loop
 */
void RagdollModel::update(float delta) {
    if (_node != nullptr) {
      std::vector<std::shared_ptr<scene2::SceneNode>> children = _node->getChildren();
      int i = 0;

	  // Update the nodes of the attached bodies
      for (auto it = children.begin(); it != children.end(); ++it) {
        (*it)->setPosition(_bodies[i]->getPosition()*_drawscale);
		(*it)->setAngle(_bodies[i]->getAngle());

		// Propagate the update to the bodies attached to the Ragdoll
        _bodies[i]->update(delta);
        i++;
      }
    }
}

/**
 * Creates the joints for this object.
 *
 * This method is executed as part of activePhysics. This is the primary method to
 * override for custom physics objects.
 *
 * @param world Box2D world to store joints
 *
 * @return true if object allocation succeeded
 */
bool RagdollModel::createJoints(b2World& world) {

	b2RevoluteJointDef jointDef;
	b2Joint* joint;

	// NECK JOINT
	jointDef.bodyA = _bodies[PART_HEAD]->getBody();
	jointDef.bodyB = _bodies[PART_BODY]->getBody();
	jointDef.localAnchorA.Set(0, (-TORSO_OFFSET) / 2);
	jointDef.localAnchorB.Set(0, (TORSO_OFFSET) / 2);
	jointDef.enableLimit = true;
	jointDef.upperAngle = M_PI / 2.0f;
	jointDef.lowerAngle = -M_PI / 2.0f;
	joint = world.CreateJoint(&jointDef);
	_joints.push_back(joint);

	// SHOULDERS
	jointDef.bodyA = _bodies[PART_LEFT_ARM]->getBody();
	jointDef.bodyB = _bodies[PART_BODY]->getBody();
	jointDef.localAnchorA.Set(ARM_XOFFSET / 2, 0);
	jointDef.localAnchorB.Set(-ARM_XOFFSET / 2, ARM_YOFFSET);
	jointDef.enableLimit = true;
	jointDef.upperAngle = M_PI / 2.0f;
	jointDef.lowerAngle = -M_PI / 2.0f;
	joint = world.CreateJoint(&jointDef);
	_joints.push_back(joint);

	jointDef.bodyA = _bodies[PART_RIGHT_ARM]->getBody();
	jointDef.bodyB = _bodies[PART_BODY]->getBody();
	jointDef.localAnchorA.Set(-ARM_XOFFSET / 2, 0);
	jointDef.localAnchorB.Set(ARM_XOFFSET / 2, ARM_YOFFSET);
	jointDef.enableLimit = true;
	jointDef.upperAngle = M_PI / 2.0f;
	jointDef.lowerAngle = -M_PI / 2.0f;
	joint = world.CreateJoint(&jointDef);
	_joints.push_back(joint);

	// ELBOWS
	jointDef.bodyA = _bodies[PART_LEFT_FOREARM]->getBody();
	jointDef.bodyB = _bodies[PART_LEFT_ARM]->getBody();
	jointDef.localAnchorA.Set(FOREARM_OFFSET / 2, 0);
	jointDef.localAnchorB.Set(-FOREARM_OFFSET / 2, 0);
	jointDef.enableLimit = true;
	jointDef.upperAngle = M_PI / 2;
	jointDef.lowerAngle = -M_PI / 2;
	joint = world.CreateJoint(&jointDef);
	_joints.push_back(joint);

	jointDef.bodyA = _bodies[PART_RIGHT_FOREARM]->getBody();
	jointDef.bodyB = _bodies[PART_RIGHT_ARM]->getBody();
	jointDef.localAnchorA.Set(-FOREARM_OFFSET / 2, 0);
	jointDef.localAnchorB.Set(FOREARM_OFFSET / 2, 0);
	jointDef.enableLimit = true;
	jointDef.upperAngle = M_PI / 2;
	jointDef.lowerAngle = -M_PI / 2;
	joint = world.CreateJoint(&jointDef);
	_joints.push_back(joint);

	// HIPS
	jointDef.bodyA = _bodies[PART_LEFT_THIGH]->getBody();
	jointDef.bodyB = _bodies[PART_BODY]->getBody();
	jointDef.localAnchorA.Set(0, THIGH_YOFFSET / 2);
	jointDef.localAnchorB.Set(-THIGH_XOFFSET, -THIGH_YOFFSET / 2);
	jointDef.enableLimit = true;
	jointDef.upperAngle = M_PI / 2;
	jointDef.lowerAngle = -M_PI / 2;
	joint = world.CreateJoint(&jointDef);
	_joints.push_back(joint);

	jointDef.bodyA = _bodies[PART_RIGHT_THIGH]->getBody();
	jointDef.bodyB = _bodies[PART_BODY]->getBody();
	jointDef.localAnchorA.Set(0, THIGH_YOFFSET / 2);
	jointDef.localAnchorB.Set(THIGH_XOFFSET, -THIGH_YOFFSET / 2);
	jointDef.enableLimit = true;
	jointDef.upperAngle = M_PI / 2;
	jointDef.lowerAngle = -M_PI / 2;
	joint = world.CreateJoint(&jointDef);
	_joints.push_back(joint);

	// KNEES
	jointDef.bodyA = _bodies[PART_LEFT_THIGH]->getBody();
	jointDef.bodyB = _bodies[PART_LEFT_SHIN]->getBody();
	jointDef.localAnchorA.Set(0, -SHIN_OFFSET / 2);
	jointDef.localAnchorB.Set(0, SHIN_OFFSET / 2);
	jointDef.enableLimit = true;
	jointDef.upperAngle = M_PI / 2;
	jointDef.lowerAngle = -M_PI / 2;
	joint = world.CreateJoint(&jointDef);
	_joints.push_back(joint);

	jointDef.bodyA = _bodies[PART_RIGHT_THIGH]->getBody();
	jointDef.bodyB = _bodies[PART_RIGHT_SHIN]->getBody();
	jointDef.localAnchorA.Set(0, -SHIN_OFFSET / 2);
	jointDef.localAnchorB.Set(0, SHIN_OFFSET / 2);
	jointDef.enableLimit = true;
	jointDef.upperAngle = M_PI / 2;
	jointDef.lowerAngle = -M_PI / 2;
	joint = world.CreateJoint(&jointDef);
	_joints.push_back(joint);

	// Weld bubbler to the head.
	b2WeldJointDef weldDef;
	return true;
}

#pragma mark -
#pragma mark Scene Graph Management

/**
 * Sets the scene graph node representing this Ragdoll.
 *
 * Note that this method also handles creating the nodes for the body parts
 * of this Ragdoll. Since the obstacles are decoupled from the scene graph,
 * initialization (which creates the obstacles) occurs prior to the call to
 * this method. Therefore, to be drawn to the screen, the nodes of the attached
 * bodies must be added here.
 *
 * The bubbler also uses the world node when adding bubbles to the scene, so
 * the input node must be added to the world BEFORE this method is called.
 *
 * By storing a reference to the scene graph node, the model can update
 * the node to be in sync with the physics info. It does this via the
 * {@link Obstacle#update(float)} method.
 *
 * @param node  The scene graph node representing this Ragdoll, which has been added to the world node already.
 */
void RagdollModel::setSceneNode(const std::shared_ptr<cugl::scene2::SceneNode>& node){
	_node = node;
	for (int ii = 0; ii <= PART_RIGHT_SHIN; ii++) {
        std::shared_ptr<Texture> image = _textures[ii];
		std::shared_ptr<scene2::PolygonNode> sprite = scene2::PolygonNode::allocWithTexture(image);
		if (ii == PART_RIGHT_ARM || ii == PART_RIGHT_FOREARM) {
			sprite->flipHorizontal(true); // More reliable than rotating 90 degrees.
		}
		_node->addChild(sprite);
	}
}

/**
 * Sets the ratio of the Ragdoll sprite to the physics body
 *
 * The Ragdoll needs this value to convert correctly between the physics
 * coordinates and the drawing screen coordinates.  Otherwise it will
 * interpret one Box2D unit as one pixel.
 *
 * All physics scaling must be uniform.  Rotation does weird things when
 * attempting to scale physics by a non-uniform factor.
 *
 * @param scale The ratio of the Ragdoll sprite to the physics body
 */
void RagdollModel::setDrawScale(float scale) {
    _drawscale = scale;
}

