#include "MuJoCo_node.h"

// -----------------------------------------------------------------------------------------
// Keyboard + mouse callbacks + variables
void scroll(GLFWwindow* window, double xoffset, double yoffset);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
void windowCloseCallback(GLFWwindow * /*window*/) ;

bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;
// ------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------
// MuJoCo Visualisation variables
mjvCamera cam;                   // abstract camera
mjvScene scn;                   // abstract scene
mjvOption opt;			        // visualization options
mjrContext con;				    // custom GPU context
GLFWwindow *window;              // GLFW window
// ----------------------------------------------------------------------

mjModel *model;
mjData* mdata_real;

MuJoCo_realRobot_ROS* mujoco_realRobot_ROS;

void setupMujocoWorld(){
    char error[1000];

    // TODO - fix this hard coded path issue
<<<<<<< HEAD
    model = mj_loadXML("/home/davidrussell/catkin_ws/src/MuJoCo_realRobot_ROS/models/franka_emika_panda/reaching_scene.xml", NULL, error, 1000);
    //model = mj_loadXML("/home/davidrussell/catkin_ws/src/MuJoCo_realRobot_ROS/models/old/reaching.xml", NULL, error, 1000);
=======
    // model = mj_loadXML("/home/davidrussell/catkin_ws/src/MuJoCo_realRobot_ROS/models/franka_emika_panda/reaching_scene.xml", NULL, error, 1000);
    model = mj_loadXML("/home/davidrussell/catkin_ws/src/MuJoCo_realRobot_ROS/models/old/reaching.xml", NULL, error, 1000);
>>>>>>> dabdf0409c02e3ffebf9e0b6ca6baefbef9e6d40

    if(!model) {
        std::cout << "model xml Error" << std::endl;
        printf("%s\n", error);
    }

    // make data corresponding to model
    mdata_real = mj_makeData(model);

    // init GLFW, create window, make OpenGL context current, request v-sync
    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    window = glfwCreateWindow(1200, 900, "iLQR_Testing", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);

    // cam.distance = 1.485;
    // cam.azimuth = 178.7;
    // cam.elevation = -31.3;
    // cam.lookat[0] = 0.325;
    // cam.lookat[1] = -0.0179;
    // cam.lookat[2] = 0.258;

    // // create scene and context
    mjv_makeScene(model, &scn, 2000);
    mjr_makeContext(model, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    glfwSetWindowCloseCallback(window, windowCloseCallback);
}

void render(){

    mujoco_realRobot_ROS->updateMujocoData(model, mdata_real);

    // get framebuffer viewport
    mjrRect viewport = { 0, 0, 0, 0 };
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(model, mdata_real, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

int main(int argc, char **argv){
    std::cout << "Today is the 17th November" << std::endl;

    setupMujocoWorld();

    // Create an instance of 
    // MuJoCo_realRobot_ROS mujocoController(true, &n);
    mujoco_realRobot_ROS = new MuJoCo_realRobot_ROS(argc, argv, 2);

    int counter = 0;

    mujoco_realRobot_ROS->switchController("effort_group_position_controller");

    double desiredPos[7];

    while(!mujoco_realRobot_ROS->firstCallbackCalled){
        ros::spinOnce();

    }

    for(int i = 0; i < 7; i++){
        desiredPos[i] = mujoco_realRobot_ROS->jointVals[i];
        std::cout << "current pos: " << desiredPos[i] << std::endl;
    }
    int testJoint = 6;

    desiredPos[testJoint] += 0.1;

    while(ros::ok()){

        counter++;
        if(counter < 200){
<<<<<<< HEAD

            //mujoco_realRobot_ROS->sendPositionsToRealRobot(desiredPos);
=======
            double torques[7] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            mujoco_realRobot_ROS->sendTorquesToRealRobot(torques);
>>>>>>> dabdf0409c02e3ffebf9e0b6ca6baefbef9e6d40
        }

        std::cout << "test Joint diff: " << desiredPos[testJoint] - mujoco_realRobot_ROS->jointVals[testJoint] << std::endl;

        render();
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------
//
//              Camera viewing callbacks using mouse and keyboard
//
//---------------------------------------------------------------------------------------------------

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods){

}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods){
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos){
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(model, action, dx / height, dy / height, &scn, &cam);

}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset){
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void windowCloseCallback(GLFWwindow * /*window*/) {
    // Use this flag if you wish not to terminate now.
    // glfwSetWindowShouldClose(window, GLFW_FALSE);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(mdata_real);
    mj_deleteModel(model);
    mj_deactivate();
}