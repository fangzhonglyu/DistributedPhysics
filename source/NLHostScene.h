//
//  NLHostScene.h
//  Network Lab
//
//  This class represents the scene for the host when creating a game. Normally
//  this class would be combined with the class for the client scene (as both
//  initialize the network controller).  But we have separated to make the code
//  a little clearer for this lab.
//
//  Author: Walker White, Aidan Hobler
//  Version: 2/8/22
//
#ifndef __NL_HOST_SCENE_H__
#define __NL_HOST_SCENE_H__
#include <cugl/cugl.h>
#include <vector>

#define PING_TEST_COUNT 5

using namespace cugl::netphysics;

/**
 * This class provides the interface to make a new game.
 *
 * Most games have a since "matching" scene whose purpose is to initialize the
 * network controller.  We have separate the host from the client to make the
 * code a little more clear.
 */
class HostScene : public cugl::Scene2 {

protected:
    /** The asset manager for this scene. */
    std::shared_ptr<cugl::AssetManager> _assets;
    /** The network connection (as made by this scene) */
    std::shared_ptr<NetEventController> _network;

    /** The menu button for starting a game */
    std::shared_ptr<cugl::scene2::Button> _startgame;
    /** The back button for the menu scene */
    std::shared_ptr<cugl::scene2::Button> _backout;
    /** The game id label (for updating) */
    std::shared_ptr<cugl::scene2::Label> _gameid;
    /** The players label (for updating) */
    std::shared_ptr<cugl::scene2::Label> _player;

    bool clicked = false;
    
    /** The network configuration */
    cugl::net::NetcodeConfig _config;
    
    cugl::Timestamp _pingTimer;
    
    Uint64 _totalPing;
    
    int _sendCount;
    
    int _receiveCount;

public:
#pragma mark -
#pragma mark Constructors
    /**
     * Creates a new host scene with the default values.
     *
     * This constructor does not allocate any objects or start the game.
     * This allows us to use the object without a heap pointer.
     */
    HostScene() : cugl::Scene2() {}
    
    /**
     * Disposes of all (non-static) resources allocated to this mode.
     *
     * This method is different from dispose() in that it ALSO shuts off any
     * static resources, like the input controller.
     */
    ~HostScene() { dispose(); }
    
    /**
     * Disposes of all (non-static) resources allocated to this mode.
     */
    void dispose() override;
    
    /**
     * Initializes the controller contents, and starts the game
     *
     * In previous labs, this method "started" the scene.  But in this
     * case, we only use to initialize the scene user interface.  We
     * do not activate the user interface yet, as an active user
     * interface will still receive input EVEN WHEN IT IS HIDDEN.
     *
     * That is why we have the method {@link #setActive}.
     *
     * @param assets    The (loaded) assets for this game mode
     *
     * @return true if the controller is initialized properly, false otherwise.
     */
    bool init(const std::shared_ptr<cugl::AssetManager>& assets, std::shared_ptr<NetEventController> network);
    
    /**
     * Sets whether the scene is currently active
     *
     * This method should be used to toggle all the UI elements.  Buttons
     * should be activated when it is made active and deactivated when
     * it is not.
     *
     * @param value whether the scene is currently active
     */
    virtual void setActive(bool value) override;

    /**
     * The method called to update the scene.
     *
     * We need to update this method to constantly talk to the server
     *
     * @param timestep  The amount of time (in seconds) since the last frame
     */
    void update(float timestep) override;
    
    /**
     * Disconnects this scene from the network controller.
     *
     * Technically, this method does not actually disconnect the network controller.
     * Since the network controller is a smart pointer, it is only fully disconnected
     * when ALL scenes have been disconnected.
     */
    void disconnect() { _network->disconnect(); }

private:
    /**
     * Updates the text in the given button.
     *
     * Techincally a button does not contain text. A button is simply a scene graph
     * node with one child for the up state and another for the down state. So to
     * change the text in one of our buttons, we have to descend the scene graph.
     * This method simplifies this process for you.
     *
     * @param button    The button to modify
     * @param text      The new text value
     */
    void updateText(const std::shared_ptr<cugl::scene2::Button>& button, const std::string text);
    
};

#endif /* __NL_HOST_SCENE_H__ */
