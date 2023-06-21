//
//  RDApp.cpp
//  Rocket Demo
//
//  This is the root class for your game.  The file main.cpp accesses this class
//  to run the application.  While you could put most of your game logic in
//  this class, we prefer to break the game up into player modes and have a
//  class for each mode.
//
//  Author: Walker White
//  Version: 1/10/17
//
#include "RDApp.h"

using namespace cugl;


#pragma mark -
#pragma mark Application State

/**
 * The method called after OpenGL is initialized, but before running the application.
 *
 * This is the method in which all user-defined program intialization should
 * take place.  You should not create a new init() method.
 *
 * When overriding this method, you should call the parent method as the
 * very last line.  This ensures that the state will transition to FOREGROUND,
 * causing the application to run.
 */
void RocketApp::onStartup() {
    _assets = AssetManager::alloc();
    _batch  = SpriteBatch::alloc();
    
    // Start-up basic input
#ifdef CU_TOUCH_SCREEN
    Input::activate<Touchscreen>();
#else
    Input::activate<Mouse>();
#endif
    
    Input::activate<Keyboard>();
    Input::activate<TextInput>();
    
    _assets->attach<Font>(FontLoader::alloc()->getHook());
    _assets->attach<Texture>(TextureLoader::alloc()->getHook());
    _assets->attach<Sound>(SoundLoader::alloc()->getHook());
    _assets->attach<scene2::SceneNode>(Scene2Loader::alloc()->getHook());
    _assets->attach<JsonValue>(JsonLoader::alloc()->getHook());
    _assets->attach<WidgetValue>(WidgetLoader::alloc()->getHook());

    _loading.init(_assets);
    _status = LOAD;
    
    // Que up the other assets
    AudioEngine::start(24);
    _assets->loadDirectoryAsync("json/assets.json",nullptr);
    
    cugl::net::NetworkLayer::start(net::NetworkLayer::Log::INFO);
    
    Application::onStartup(); // YOU MUST END with call to parent
}

/**
 * The method called when the application is ready to quit.
 *
 * This is the method to dispose of all resources allocated by this
 * application.  As a rule of thumb, everything created in onStartup()
 * should be deleted here.
 *
 * When overriding this method, you should call the parent method as the
 * very last line.  This ensures that the state will transition to NONE,
 * causing the application to be deleted.
 */
void RocketApp::onShutdown() {
    _gameplay.dispose();
    _mainmenu.dispose();
    _hostgame.dispose();
    _joingame.dispose();
    _assets = nullptr;
    _batch = nullptr;
    
    // Shutdown input
#ifdef CU_TOUCH_SCREEN
    Input::deactivate<Touchscreen>();
#else
    Input::deactivate<Mouse>();
#endif
    
	AudioEngine::stop();
	Application::onShutdown();  // YOU MUST END with call to parent
}

/**
 * The method called when the application is suspended and put in the background.
 *
 * When this method is called, you should store any state that you do not
 * want to be lost.  There is no guarantee that an application will return
 * from the background; it may be terminated instead.
 *
 * If you are using audio, it is critical that you pause it on suspension.
 * Otherwise, the audio thread may persist while the application is in
 * the background.
 */
void RocketApp::onSuspend() {
    AudioEngine::get()->pause();
}

/**
 * The method called when the application resumes and put in the foreground.
 *
 * If you saved any state before going into the background, now is the time
 * to restore it. This guarantees that the application looks the same as
 * when it was suspended.
 *
 * If you are using audio, you should use this method to resume any audio
 * paused before app suspension.
 */
void RocketApp::onResume() {
    AudioEngine::get()->resume();
}


#pragma mark -
#pragma mark Application Loop

#if USING_PHYSICS

void RocketApp::preUpdate(float timestep){
    if (_status == LOAD && _loading.isActive()) {
        _loading.update(0.01f);
    }
    else if (_status == LOAD) {
        _loading.dispose(); // Disables the input listeners in this mode
        _mainmenu.init(_assets);
        _mainmenu.setActive(true);
        _hostgame.init(_assets);
        _joingame.init(_assets);
        _gameplay.init(_assets);
        _status = MENU;
    }
    else if (_status == MENU) {
        updateMenuScene(timestep);
    }
    else if (_status == HOST){
        updateHostScene(timestep);
    }
    else if (_status == CLIENT){
        updateClientScene(timestep);
    }
    else if (_status == GAME){
        _gameplay.preUpdate(timestep);
    }
}

void RocketApp::postUpdate(float timestep) {
    if (_status == GAME) {
        _gameplay.postUpdate(timestep);
    }
}

void RocketApp::fixedUpdate() {
    if (_status == GAME) {
        _gameplay.fixedUpdate();
    }
}
#else
/**
 * The method called to update the application data.
 *
 * This is your core loop and should be replaced with your custom implementation.
 * This method should contain any code that is not an OpenGL call.
 *
 * When overriding this method, you do not need to call the parent method
 * at all. The default implmentation does nothing.
 *
 * @param timestep  The amount of time (in seconds) since the last frame
 */
void RocketApp::update(float timestep) {
    //deprecated
}
#endif

/**
 * Inidividualized update method for the menu scene.
 *
 * This method keeps the primary {@link #update} from being a mess of switch
 * statements. It also handles the transition logic from the menu scene.
 *
 * @param timestep  The amount of time (in seconds) since the last frame
 */
void RocketApp::updateMenuScene(float timestep) {
    _mainmenu.update(timestep);
    switch (_mainmenu.getChoice()) {
        case MenuScene::Choice::HOST:
            _mainmenu.setActive(false);
            _hostgame.setActive(true);
            _status = HOST;
            break;
        case MenuScene::Choice::JOIN:
            _mainmenu.setActive(false);
            _joingame.setActive(true);
            _status = CLIENT;
            break;
        case MenuScene::Choice::NONE:
            // DO NOTHING
            break;
    }
}

/**
 * Inidividualized update method for the host scene.
 *
 * This method keeps the primary {@link #update} from being a mess of switch
 * statements. It also handles the transition logic from the host scene.
 *
 * @param timestep  The amount of time (in seconds) since the last frame
 */
void RocketApp::updateHostScene(float timestep) {
    _hostgame.update(timestep);
    switch (_hostgame.getStatus()) {
        case HostScene::Status::ABORT:
            _hostgame.setActive(false);
            _mainmenu.setActive(true);
            _status = MENU;
            break;
        case HostScene::Status::START:
            _hostgame.setActive(false);
            _gameplay.setActive(true);
            _status = GAME;
            // Transfer connection ownership
            _gameplay.setNetwork(_hostgame.getConnection());
            _hostgame.disconnect();
            _gameplay.setHost(true);
            break;
        case HostScene::Status::WAIT:
        case HostScene::Status::IDLE:
            // DO NOTHING
            break;
    }
}

/**
 * Inidividualized update method for the client scene.
 *
 * This method keeps the primary {@link #update} from being a mess of switch
 * statements. It also handles the transition logic from the client scene.
 *
 * @param timestep  The amount of time (in seconds) since the last frame
 */
void RocketApp::updateClientScene(float timestep) {
    _joingame.update(timestep);
    switch (_joingame.getStatus()) {
        case ClientScene::Status::ABORT:
            _joingame.setActive(false);
            _mainmenu.setActive(true);
            _status = MENU;
            break;
        case ClientScene::Status::START:
            _joingame.setActive(false);
            _gameplay.setActive(true);
            _status = GAME;
            // Transfer connection ownership
            _gameplay.setNetwork(_joingame.getConnection());
            _joingame.disconnect();
            _gameplay.setHost(false);
            break;
        case ClientScene::Status::WAIT:
        case ClientScene::Status::IDLE:
        case ClientScene::Status::JOIN:
            // DO NOTHING
            break;
    }
}

/**
 * The method called to draw the application to the screen.
 *
 * This is your core loop and should be replaced with your custom implementation.
 * This method should OpenGL and related drawing calls.
 *
 * When overriding this method, you do not need to call the parent method
 * at all. The default implmentation does nothing.
 */
void RocketApp::draw() {
    switch (_status) {
        case LOAD:
            _loading.render(_batch);
            break;
        case MENU:
            _mainmenu.render(_batch);
            break;
        case HOST:
            _hostgame.render(_batch);
            break;
        case CLIENT:
            _joingame.render(_batch);
            break;
        case GAME:
            _gameplay.render(_batch);
        default:
            break;
    }
}

