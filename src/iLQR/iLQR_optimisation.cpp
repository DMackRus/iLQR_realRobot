#include "Utility/MujocoController/MujocoUI.h"
#include "iLQR/iLQR_dataCentric.h"
#include "modelTranslator/modelTranslator.h"
#include "Utility/stdInclude/stdInclude.h"
#include "MuJoCo_node.h"
#include "mujoco.h"


// Different operating modes for my code
#define RUN_ILQR                1     // RUN_ILQR - runs a simple iLQR optimisation for given task

extern MujocoController *globalMujocoController;
extern mjModel* model;						// MuJoCo model
extern mjData* mdata;						// MuJoCo data

extern iLQR* optimiser;
frankaModel* modelTranslator;

extern mjvCamera cam;                   // abstract camera
extern mjvScene scn;                    // abstract scene
extern mjvOption opt;			        // visualization options
extern mjrContext con;				    // custom GPU context
extern GLFWwindow *window;

MuJoCo_realRobot_ROS* mujoco_realRobot_ROS;

typedef Matrix<double, (2), 1> m_cube;

m_state X0;
m_state X_desired;

extern std::vector<m_ctrl> testInitControls;
extern std::vector<bool> grippersOpen;
extern mjData* d_init_test;
extern m_point intermediatePoint;

void saveTrajecToCSV();

void simpleTest();

void updateStartGoalAndData();
void initControls();
m_state generateRandomStartState();
m_state generateRandomGoalState(m_state startState);

void sendTrajecToRealRobot();

int main(int argc, char **argv){

    bool simulateFinal = true;
    bool realRobot;

    modelTranslator = new frankaModel();
    initMujoco(modelTranslator->taskNumber, 0.004);
    modelTranslator->init(model);

    mujoco_realRobot_ROS = new MuJoCo_realRobot_ROS(argc, argv, 1);

    ros::param::get("/realRobot", realRobot);
    if(realRobot){
        simulateFinal = false;
    }

    mujoco_realRobot_ROS->switchController("effort_group_position_controller");

    d_init_test = mj_makeData(model);

    while(!mujoco_realRobot_ROS->firstCallbackCalled){
        mujoco_realRobot_ROS->updateMujocoData(model, d_init_test);
    }

    mujoco_realRobot_ROS->updateMujocoData(model, d_init_test);

    m_state startingState = modelTranslator->returnState(d_init_test);

    cout << "starting state: " << startingState << endl;

    X_desired = startingState.replicate(1, 1);

    X_desired(0) = 0.0;
    X_desired(1) = 0.0;
    X_desired(2) = -0.1;
    X_desired(3) = -1.85;
    X_desired(4) = 0.0;
    X_desired(5) = 1.95;
    X_desired(6) = 0.75;

    X_desired(7) = 0.8;
    X_desired(8) = 0.0;

    // Load initial state into main data
    cpMjData(model, mdata, d_init_test);

    if(RUN_ILQR){

        optimiser = new iLQR(model, mdata, modelTranslator, globalMujocoController);
        optimiser->makeDataForOptimisation();

        // X_desired << 0.204, 0.183, -0.914, -1.76, -0.266, 0.8, -0.7,
        //                 0, 0, 0, 0, 0, 0, 0;

        cpMjData(model, d_init_test, mdata);

        cout << "startingState: " << startingState << endl;
        cout << "X desired: " << X_desired << endl;

        modelTranslator->setDesiredState(X_desired);
        optimiser->resetInitialStates(d_init_test, X_desired);

        for(int i = 0; i < 1; i++){
            testInitControls.clear();
            auto iLQRStart = high_resolution_clock::now();
            optimiser->updateNumStepsPerDeriv(5);

            cpMjData(model, mdata, optimiser->d_init);
            initControls();
            optimiser->setInitControls(testInitControls, grippersOpen);

            optimiser->optimise();

            auto iLQRStop = high_resolution_clock::now();
            auto iLQRDur = duration_cast<microseconds>(iLQRStop - iLQRStart);

            float milliiLQRTime = iLQRDur.count()/1000;

        }
        if(simulateFinal){
            render();
        }
        else{
            sendTrajecToRealRobot();
        }
    }
    // Just want to test physics simulator things manually
    else{

        optimiser = new iLQR(model, mdata, modelTranslator, globalMujocoController);

        optimiser->resetInitialStates(d_init_test, X_desired);

        simpleTest();

        render_simpleTest();
    }

    return 0;
}

void updateStartGoalAndData(){
    X0 = modelTranslator->generateRandomStartState(mdata);
    //cout << "-------------- random init state ----------------" << endl << X0 << endl;
    X_desired = modelTranslator->generateRandomGoalState(X0, mdata);
    //cout << "-------------- random desired state ----------------" << endl << X_desired << endl;
    modelTranslator->setState(mdata, X0);

    for(int i = 0; i < 10; i++){
        mj_step(model, mdata);
    }
    cpMjData(model, d_init_test, mdata);

}

void sendTrajecToRealRobot(){
    int controlCounter = 0;
    const int realControlsPerSim = 4;
    int realControlsCounter = 0;

    cpMjData(model, mdata, d_init_test);

    ros::Rate loop_rate(1000);

    cout << "started" << endl;
    bool printOnce = false;

    while(ros::ok()){

        // Update the robot data state
        // ros spin is in here

        // mujoco_realRobot_ROS->updateMujocoData(model, mdata);
        // mj_forward(model, mdata);

        // if there are still commands to send, send them
        if(controlCounter < MUJ_STEPS_HORIZON_LENGTH){
            
            m_ctrl nextControl = optimiser->returnDesiredControl(controlCounter, true);

            double positions[7];
            for(int i = 0; i < 7; i++){
                //std::cout << "next control desired: " << nextControl(1) << " bias force: " << mdata->qfrc_bias[1] << std::endl;
                positions[i] = nextControl(i);
            }

            mujoco_realRobot_ROS->sendPositionsToRealRobot(positions);
            //cout << "position being sent - " << controlCounter << endl;
            

            realControlsCounter++;
            if(realControlsCounter >= (realControlsPerSim)){
                controlCounter++;
                realControlsCounter = 0;
                // cout << "next control: " << nextControl << endl;
            }
        }
        else{
            if(!printOnce){
                printOnce = true;
                cout << "done" << endl;
                mujoco_realRobot_ROS->updateMujocoData(model, mdata);
                m_state termState = modelTranslator->returnState(mdata);
                m_state stateDiff = X_desired - termState;
                cout << "state diff: " << stateDiff << endl;

            }
        }

        // Visualise curent robot state
        // mjrRect viewport = { 0, 0, 0, 0 };
        // glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // // update scene and render
        // mjv_updateScene(model, mdata, &opt, NULL, &cam, mjCAT_ALL, &scn);
        // mjr_render(viewport, &scn, &con);

        // // swap OpenGL buffers (blocking call due to v-sync)
        // glfwSwapBuffers(window);

        // // process pending GUI events, call GLFW callbacks
        // glfwPollEvents();

        loop_rate.sleep();

    }
}

// Init controls code for different tasks
#ifdef DOUBLE_PENDULUM
void initControls(){

    for(int i = 0; i <= MUJ_STEPS_HORIZON_LENGTH; i++){

        testInitControls.push_back(m_ctrl());
        grippersOpen.push_back(bool());
        grippersOpen[i] = false;

        for(int k = 0; k < NUM_CTRL; k++){

//            testInitControls[i](k) = mdata->qfrc_bias[k];
            testInitControls[i](k) = 0;
            mdata->ctrl[k] = testInitControls[i](k);

        }

        for(int j = 0; j < 1; j++){
            mj_step(model, mdata);
        }
    }
}

#endif

#ifdef REACHING
void initControls(){

    m_ctrl lastControl;
    lastControl.setZero();
    int jerkLimit = 1;

    m_state startState = modelTranslator->returnState(mdata);
    m_state stateDiff = X_desired - startState;
    m_state currentStateDesired = startState;

    for(int i = 0; i <= MUJ_STEPS_HORIZON_LENGTH; i++){

        m_state X_diff;
        m_state X = modelTranslator->returnState(mdata);
        m_ctrl nextControl;
        X_diff = X_desired - X;

        testInitControls.push_back(m_ctrl());
        grippersOpen.push_back(bool());
        grippersOpen[i] = false;

        for(int i = 0; i < 7; i++){
            currentStateDesired(i) += (stateDiff(i) / MUJ_STEPS_HORIZON_LENGTH);

            nextControl(i) = currentStateDesired(i);
        }

        for(int k = 0; k < NUM_CTRL; k++){

            // if(nextControl(k) - lastControl(k) > jerkLimit){
            //     nextControl(k) = lastControl(k) + jerkLimit;
            // }

            // if(nextControl(k) - lastControl(k) < -jerkLimit){
            //     nextControl(k) = lastControl(k) - jerkLimit;
            // }

            // if(nextControl(k) > modelTranslator->torqueLims[k]) nextControl(k) = modelTranslator->torqueLims[k];
            // if(nextControl(k) < -modelTranslator->torqueLims[k]) nextControl(k) = -modelTranslator->torqueLims[k];

            
            //nextControl(k) += mdata->qfrc_bias[k];

            testInitControls[i](k) = nextControl(k);
            mdata->ctrl[k] = testInitControls[i](k);

        }

        lastControl = nextControl.replicate(1,1);

        for(int j = 0; j < 1; j++){
            mj_step(model, mdata);
        }
    }
}

#endif

#ifdef OBJECT_PUSHING
// void initControls() {
//     cpMjData(model, mdata, d_init_test);

//     const std::string EE_Name = "hand";
//     const std::string goalName = "tin";

//     int EE_id = mj_name2id(model, mjOBJ_BODY, EE_Name.c_str());
//     int goal_id = mj_name2id(model, mjOBJ_BODY, goalName.c_str());

//     m_pose startPose = globalMujocoController->returnBodyPose(model, mdata, EE_id);
//     m_quat startQuat = globalMujocoController->returnBodyQuat(model, mdata, EE_id);

//     // TODO hard coded - get it programmatically? - also made it slightly bigger so trajectory has room to improve
//     float cylinder_radius = 0.04;
//     float x_cylinder0ffset = cylinder_radius * sin(PI/4);
//     float y_cylinder0ffset = cylinder_radius * cos(PI/4);

//     float endPointX = X_desired(7) - x_cylinder0ffset;
//     float endPointY;
//     if(X_desired(8) - startPose(0) > 0){
//         endPointY = X_desired(8) - y_cylinder0ffset;
//     }
//     else{
//         endPointY = X_desired(8) + y_cylinder0ffset;
//     }

//     float cylinderObjectX = X0(7);
//     float cylinderObjectY = X0(8);
//     float intermediatePointX;
//     float intermediatePointY;

//     float angle = atan2(endPointY - cylinderObjectY, endPointX - cylinderObjectX);

//     if(endPointY > cylinderObjectY){
//         angle += 0.4;
//     }
//     else{
//         angle -= 0.4;
//     }

//     float h = 0.15;

//     float deltaX = h * cos(angle);
//     float deltaY = h * sin(angle);

//     // Calculate intermediate waypoint to position end effector behind cube such that it can be pushed to desired goal position
// //    if(X_desired(8) - startPose(0) > 0){
// //        intermediatePointY = cylinderObjectY + deltaY;
// //    }
// //    else{
// //        intermediatePointY = cylinderObjectY - deltaY;
// //    }
//     intermediatePointY = cylinderObjectY - deltaY;
//     intermediatePointX = cylinderObjectX - deltaX;

//     intermediatePoint(0) = intermediatePointX;
//     intermediatePoint(1) = intermediatePointY;

// //    if(endPointY - cylinderObjectY > 0){
// //        intermediatePointY -= 0.1;
// //    }
// //    else{
// //        intermediatePointY += 0.1;
// //    }

//     float x_diff = intermediatePointX - startPose(0);
//     float y_diff = intermediatePointY - startPose(1);

//     m_point initPath[MUJ_STEPS_HORIZON_LENGTH];
//     initPath[0](0) = startPose(0);
//     initPath[0](1) = startPose(1);
//     initPath[0](2) = startPose(2);

//     int splitIndex = 1000;

//     for (int i = 0; i < 1000; i++) {
//         initPath[i + 1](0) = initPath[i](0) + (x_diff / splitIndex);
//         initPath[i + 1](1) = initPath[i](1) + (y_diff / splitIndex);
//         initPath[i + 1](2) = initPath[i](2);
//     }

//     x_diff = endPointX - intermediatePointX;
//     y_diff = endPointY - intermediatePointY;

//     // Deliberately make initial;isation slightly worse so trajectory optimiser can do something
//     // for (int i = splitIndex; i < MUJ_STEPS_HORIZON_LENGTH - 1; i++) {
//     //     initPath[i + 1](0) = initPath[i](0) + (x_diff / (5000 - splitIndex));
//     //     initPath[i + 1](1) = initPath[i](1) + (y_diff / (5000 - splitIndex));
//     //     initPath[i + 1](2) = initPath[i](2);
//     // }

//    for (int i = splitIndex; i < MUJ_STEPS_HORIZON_LENGTH - 1; i++) {
//        initPath[i + 1](0) = initPath[i](0) + (x_diff / (MUJ_STEPS_HORIZON_LENGTH - splitIndex));
//        initPath[i + 1](1) = initPath[i](1) + (y_diff / (MUJ_STEPS_HORIZON_LENGTH - splitIndex));
//        initPath[i + 1](2) = initPath[i](2);
//    }
   
//     m_state currentState = modelTranslator->returnState(mdata);
//     m_ctrl lastDesiredPos;

//     for(int i = 0; i < NUM_CTRL; i++){
//         lastDesiredPos(i) = currentState(i);
//     }

//     testInitControls.push_back(m_ctrl());
//     grippersOpen.push_back(bool());
//     grippersOpen[0] = false;

//     for (int k = 0; k < NUM_CTRL; k++) {

//         testInitControls[0](k) = lastDesiredPos(k);
//         mdata->ctrl[k] = testInitControls[0](k);
//     }

//     mj_step(model, mdata);

//     for (int i = 1; i <= MUJ_STEPS_HORIZON_LENGTH; i++) {

//         testInitControls.push_back(m_ctrl());
//         grippersOpen.push_back(bool());
//         grippersOpen[i] = false;

//         for (int k = 0; k < NUM_CTRL; k++) {

//             testInitControls[i](k) = lastDesiredPos(k);
//             mdata->ctrl[k] = testInitControls[i](k);
//         }

//         m_pose currentEEPose = globalMujocoController->returnBodyPose(model, mdata, EE_id);
//         m_quat currentQuat = globalMujocoController->returnBodyQuat(model, mdata, EE_id);

//         m_quat invQuat = globalMujocoController->invQuat(currentQuat);
//         m_quat quatDiff = globalMujocoController->multQuat(startQuat, invQuat);

//         m_point axisDiff = globalMujocoController->quat2Axis(quatDiff);

//         m_pose differenceFromPath;
//         for (int j = 0; j < 3; j++) {
//             differenceFromPath(j) = initPath[i](j) - initPath[i-1](j);
//             differenceFromPath(j + 3) = 0;
//         }

//         MatrixXd Jac = globalMujocoController->calculateJacobian(model, mdata, EE_id);

//         MatrixXd Jac_t = Jac.transpose();
//         MatrixXd Jac_inv = Jac.completeOrthogonalDecomposition().pseudoInverse();

//         m_ctrl delta_q = Jac_inv * differenceFromPath;

//         lastDesiredPos -= delta_q;

//         if(i % 50 == 0){
//             cout << "------------------------------------------------------------------------" << endl;
//             cout << "differenceFromPath" << differenceFromPath << endl;
//             cout << "jac: " << Jac_inv << endl;
//             cout << "delta q " << delta_q << endl;
//             cout << "test init controls  " << i << ": " << testInitControls[i] << endl;
//         }

// // //        cout << "current path point: " << initPath[i] << endl;
// // //        cout << "currentEE Pose: " << currentEEPose << endl;
// // //        cout << "diff from path: " << differenceFromPath << endl;

// //         m_pose desiredEEForce;

// //         for (int j = 0; j < 6; j++) {
// //             desiredEEForce(j) = differenceFromPath(j) * gains[j];
// //         }

// //         //cout << "desiredEEForce " << desiredEEForce << endl;

        

// //         m_ctrl desiredControls;

// //         desiredControls = Jac_inv * desiredEEForce;

//         for (int j = 0; j < 1; j++) {
//             mj_step(model, mdata);
//         }
//     }
// }

void initControls(){

    m_ctrl finalPos;
    finalPos << -0.16, 0.6, 0.120, -2.2, -0.08, 2.67, 0.723;

    m_ctrl start;
    m_state currentState = modelTranslator->returnState(mdata);

    for(int i = 0; i < NUM_CTRL; i++){
        start(i) = currentState(i);
    }

    m_ctrl diff = (finalPos - start) / MUJ_STEPS_HORIZON_LENGTH;

    m_ctrl nextPos = start.replicate(1, 1);


    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){

        testInitControls.push_back(m_ctrl());
        grippersOpen.push_back(bool());
        grippersOpen[0] = false;

        for (int k = 0; k < NUM_CTRL; k++) {

            testInitControls[i](k) = nextPos(k);
            mdata->ctrl[k] = testInitControls[i](k);
        }

        nextPos += diff;

        mj_step(model, mdata);

    }
}
#endif

#ifdef REACHING_CLUTTER
void initControls(){
    cpMjData(model, mdata, d_init_test);

    int grippersOpenIndex = 1000;
    const std::string endEffecName = "end_effector";
    //panda0_leftfinger
    const std::string EE_Name = "panda0_leftfinger";
    const std::string goalName = "goal";

    int pos_EE_id = mj_name2id(model, mjOBJ_SITE, endEffecName.c_str());
    int EE_id = mj_name2id(model, mjOBJ_BODY, EE_Name.c_str());
    int goal_id = mj_name2id(model, mjOBJ_BODY, goalName.c_str());

    cout << "EE id: " << EE_id << endl;
    cout << "pos_EE_id id: " << pos_EE_id << endl;

    m_point startPoint = modelTranslator->returnEE_point(mdata);
            //globalMujocoController->returnSitePoint(model, mdata, pos_EE_id);
    m_quat startQuat = globalMujocoController->returnBodyQuat(model, mdata, EE_id);
    float endPointX = X_desired(7) + 0.1;
    float endPointY = X_desired(8);

    endPointY += 0.3;

    cout << "start pose: " << startPoint << endl;
    cout << "start quart: " << startQuat << endl;

    float x_diff = endPointX - startPoint(0);
    float y_diff = endPointY - startPoint(1);

    m_point initPath[MUJ_STEPS_HORIZON_LENGTH];
    initPath[0](0) = startPoint(0);
    initPath[0](1) = startPoint(1);
    initPath[0](2) = startPoint(2);

    for (int i = 0; i < MUJ_STEPS_HORIZON_LENGTH - 1; i++) {
        initPath[i + 1](0) = initPath[i](0) + (x_diff / (MUJ_STEPS_HORIZON_LENGTH));
        initPath[i + 1](1) = initPath[i](1) + (y_diff / (MUJ_STEPS_HORIZON_LENGTH));
        initPath[i + 1](2) = initPath[i](2);
    }

    for (int i = 0; i <= MUJ_STEPS_HORIZON_LENGTH; i++) {

        m_point currentEEPoint = globalMujocoController->returnSitePoint(model, mdata, pos_EE_id);
        m_quat currentQuat = globalMujocoController->returnBodyQuat(model, mdata, EE_id);

        m_quat invQuat = globalMujocoController->invQuat(currentQuat);
        m_quat quatDiff = globalMujocoController->multQuat(startQuat, invQuat);

        m_point axisDiff = globalMujocoController->quat2Axis(quatDiff);

        m_pose differenceFromPath;
        float gains[6] = {1000, 1000, 1000, 80, 80, 80};
        for (int j = 0; j < 3; j++) {
            differenceFromPath(j) = initPath[i](j) - currentEEPoint(j);
            differenceFromPath(j + 3) = axisDiff(j);
        }

//        cout << "current path point: " << initPath[i] << endl;S
//        cout << "currentEE Pose: " << currentEEPose << endl;
//        cout << "diff from path: " << differenceFromPath << endl;

        m_pose desiredEEForce;

        for (int j = 0; j < 6; j++) {
            desiredEEForce(j) = differenceFromPath(j) * gains[j];
        }

        //cout << "desiredEEForce " << desiredEEForce << endl;

        MatrixXd Jac = globalMujocoController->calculateJacobian(model, mdata, EE_id);

        MatrixXd Jac_t = Jac.transpose();
        MatrixXd Jac_inv = Jac.completeOrthogonalDecomposition().pseudoInverse();

        m_ctrl desiredControls;

        MatrixXd jacobianControls = Jac_inv * desiredEEForce;

        testInitControls.push_back(m_ctrl());
        grippersOpen.push_back(bool());

        jacobianControls(6) = 0;


        for (int k = 0; k < NUM_CTRL; k++) {

            testInitControls[i](k) = jacobianControls(k) + mdata->qfrc_bias[k];

            mdata->ctrl[k] = testInitControls[i](k);
        }

        if(i < grippersOpenIndex){
            grippersOpen[i] = false;

        }
        else if(i > grippersOpenIndex and i < 2500){
            grippersOpen[i] = true;
        }
        else{
            grippersOpen[i] = false;
        }

        if(grippersOpen[i]){
            mdata->ctrl[7] = GRIPPERS_OPEN;
            mdata->ctrl[8] = GRIPPERS_OPEN;
        }
        else{
            mdata->ctrl[7] = GRIPPERS_CLOSED;
            mdata->ctrl[8] = GRIPPERS_CLOSED;
        }

        for (int j = 0; j < 1; j++) {
            mj_step(model, mdata);
        }
    }
}
#endif

void simpleTest(){
    initControls();
}

