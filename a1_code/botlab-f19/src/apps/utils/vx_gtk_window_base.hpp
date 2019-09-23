#ifndef APPS_UTILS_VX_GTK_WINDOW_BASE_H
#define APPS_UTILS_VX_GTK_WINDOW_BASE_H

#include <vx/vx_display.h>
#include <vx/vx_event_handler.h>
#include <apps/utils/vx_utils.hpp>
#include <common/point.hpp>
#include <mutex>

/**
* VxGtkWindowBase is a abstract base class for creating Gtk-based Vx windows. Using Vx with Gtk involves a fair amount
* of boilerplate code to get the window to appear and to create handlers for keyboard and mouse events.
*
* VxGtkWindowBase will initialize a Gtk window and create the appropriate event handlers in its constructor. After
* construction, the run method is used to start the window. This method returns when the Gtk window is closed.
*
* After run is called, a Gtk window will appear and onMouseEvent and onKeyEvents will start firing.
*/
class VxGtkWindowBase {
public:
    /**
    * Constructor for VxGtkWindowBase.
    *
    * \param    argc                    Count of command-line arguments for the program
    * \param    argv                    Command-line arguments for the program
    * \param    widthInPixels           Initial width of the GTK window to create
    * \param    heightInPixels          Initial height of the GTK window to create
    * \param    framesPerSecond         Number of frames per second to render visualizations
    *
    * \pre argc > 0
    * \pre arcv != nullptr
    * \pre widthInPixels > 0
    * \pre heightInPixels > 0
    * \pre framesPerSecond > 0
    */
    VxGtkWindowBase(int argc, char** argv, int widthInPixels, int heightInPixels, int framesPerSecond);

    /**
    * Destructor for VxGtkWindowBase.
    */
    virtual ~VxGtkWindowBase(void);

    /**
    * run launches the Gtk main thread and starts a render thread that updates at the specified number of frames per
    * second. The run method will exit when the Gtk window is closed by the user.
    *
    * run should only be called once the GUI application initialization, like LCM message subscription, is completed
    * because it blocks until the GUIwindow is closed.
    */
    void run(void);

    /**
    * onMouseEvent is fired whenever a mouse event is generated by the Gtk window.
    *
    * Mouse events are fired for the following actions:
    *
    *   - moving the mouse
    *   - pressing a mouse button
    *   - releasing a mouse button
    *
    * \see  vx/vx_event.h for the contents of a vx_mouse_event_t instance.
    *
    * \param    layer               Vx layer for which the event occurred
    * \param    cameraPosition      Current position of the Vx camera
    * \param    event               Information about the mouse event
    * \param    worldPoint          Position of the click in the world coordinate frame (not screen coords)
    * \return   1 if the event was consumed. 0 if the event was ignored and should be passed any additional handlers.
    */
    virtual int onMouseEvent(vx_layer_t* layer,
                             vx_camera_pos_t* cameraPosition,
                             vx_mouse_event_t* event,
                             Point<float> worldPoint);

    /**
    * onKeyEvent is fired whenever a keyboard event is generated by the Gtk window.
    *
    * Keyboard events are fired for the following actions:
    *
    *   - key pressed
    *   - key released
    *
    * \see  vx/vx_event.h for the contents of a vx_key_event_t instance.
    *
    * \param    layer               Vx layer for which the event occurred
    * \param    event               Information about the key event
    * \return   1 if the event was consumed. 0 if the event was ignored and should be passed any additional handlers.
    */
    virtual int onKeyEvent(vx_layer_t* layer, vx_key_event_t* event);

    /**
    * onDisplayStart is fired when the GUI display is initialized. The vx_display_t on which all rendered will be done
    * is provided.
    * 
    * \param    display         Vx display on which all rendering will occur
    */
    virtual void onDisplayStart(vx_display_t* display);

    /**
    * onDisplayFinish is fired when the GUI display is closed. The vx_display_t that was being used by the application
    * is provided.
    * 
    * \param    display         Vx display that was used for the application
    */
    virtual void onDisplayFinish(vx_display_t* display);

    /**
    * render is called for each iteration of the run-loop.
    *
    * The render method is called from the run thread, which will be a different from the thread LCM messages arrive on.
    * As a result, any LCM-based data will need to be locked with a mutex before being read for rendering.
    */
    virtual void render(void) = 0;

    /**
    * initialize is the initialization function for the GUI. It is called before onDisplayStart.
    */
    void initialize(vx_display_t* display);

protected:
    vx_world_t* world_;  ///< where vx objects live for rendering

    /**
    * createGuiLayout creates the layout for the widgets in the GUI. The default layout is a Vx rendering widget that
    * takes up the entire window. If something more is needed, then override this method to create a more sophisticated
    * GUI layout.
    * 
    * \param    window      Top-level Gtk window
    * \param    vxCanvas    Gtk widget that is the OpenGL canvas to which all Vx visualizations will be rendered
    */
    virtual void createGuiLayout(GtkWidget* window, GtkWidget* vxCanvas);

private:
    vx_application_t application_;
    vx_event_handler_t eventHandler_;
    vx_layer_t* layer_;
    std::mutex vxLock_;

    int width_;
    int height_;
    int framesPerSecond_;

    bool isInitialized_;
    bool isRunning_;

    friend void* render_loop(void* data);
};

// Friend function for the render thread
void* render_loop(void* data);

#endif  // APPS_UTILS_VX_GTK_WINDOW_BASE_H
