//
// Created by Junny Kim on 07/02/20.
// MIT License
//
// Copyright (c) 2019-2020 DRCD Lab, KAIST
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <raisim/OgreVis.hpp>
#include "raisimBasicImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp"
#include "helper.hpp"
#include "SensorSimulation.hpp"
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>


void setupCallback() {
    auto vis = raisim::OgreVis::get();

    /// light
    vis->getLight()->setDiffuseColour(1, 1, 1);
    vis->getLight()->setCastShadows(true);
    Ogre::Vector3 lightdir(-3, -3, -0.5);
    lightdir.normalise();
    vis->getLightNode()->setDirection({lightdir});

    /// load  textures
    vis->addResourceDirectory(vis->getResourceDir() + "/material/checkerboard");
    vis->loadMaterialFile("checkerboard.material");

    /// shdow setting
    vis->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
    vis->getSceneManager()->setShadowTextureSettings(2048, 3);

    /// scale related settings!! Please adapt it depending on your map size
    // beyond this distance, shadow disappears
    vis->getSceneManager()->setShadowFarDistance(30);
    // size of contact points and contact forces
    vis->setContactVisObjectSize(0.06, .6);
    // speed of camera motion in freelook mode
    vis->getCameraMan()->setTopSpeed(5);
}

int main(int argc, char **argv) {
    /// create raisim world
    raisim::World world;
    double dt = 0.0001;
    world.setTimeStep(dt);

    auto vis = raisim::OgreVis::get();

    /// these method must be called before initApp
    vis->setWorld(&world);
    vis->setWindowSize(1800, 1200);
    vis->setImguiSetupCallback(imguiSetupCallback);
    vis->setImguiRenderCallback(imguiRenderCallBack);
    vis->setKeyboardCallback(raisimKeyboardCallback);
    vis->setSetUpCallback(setupCallback);
    vis->setAntiAliasing(2);
    vis->setDesiredFPS(25);

    raisim::gui::manualStepping = false;

    /// starts visualizer thread
    vis->initApp();

    /// create raisim objects
    auto ground = world.addGround();

    /// create visualizer objects
    vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");

    /// laikago joint PD controller
    Eigen::VectorXd jointNominalConfig(19), jointVelocityTarget(18);
    Eigen::VectorXd jointState(18), jointForce(18), jointPgain(18), jointDgain(18);
    jointPgain.setZero();
    jointDgain.setZero();
    jointVelocityTarget.setZero();
    jointPgain.tail(12).setConstant(200.0);
    jointDgain.tail(12).setConstant(10.0);

    const size_t N = 8;

    auto aliengo = world.addArticulatedSystem(raisim::loadResource("aliengo/aliengo.urdf"));
    auto aliengoVis = vis->createGraphicalObject(aliengo, "aliengo");
    aliengo->setGeneralizedCoordinate({0, 0, 0.48, 1.0, 0.0, 0.0, 0.0, 0.0, 0.5, -1, 0, 0.5, -1,
                                       0.00, 0.5, -1, 0, 0.5, -0.7});
    aliengo->setGeneralizedForce(Eigen::VectorXd::Zero(aliengo->getDOF()));
    aliengo->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    aliengo->setPdGains(jointPgain, jointDgain);
    aliengo->setName("aliengo");

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 0.001);
    std::srand(std::time(nullptr));
    aliengo->printOutBodyNamesInOrder();
    aliengo->printOutFrameNamesInOrder();



    SensorSimulation::ImuSimulation imu_simulation;


    // lambda function for the controller
    auto controller = [&aliengo, &generator, &distribution, &dt, &world, &imu_simulation]() {
        static size_t controlDecimation = 0;







//        aliengo->getAngularVelocity(aliengo->getBodyIdx("base"),bodyangularvelocity);
//        aliengo->getVelocity(aliengo->getBodyIdx("base"),bodyvelocity);
//        aliengo->getBodyPose(aliengo->getBodyIdx("base"),bodyrotation,bodyposition);

        aliengo->getFramePosition(aliengo->getFrameIdxByName("imu_joint"),imu_simulation.imu_Position);
        aliengo->getFrameVelocity(aliengo->getFrameIdxByName("imu_joint"),imu_simulation.imu_Velocity);
        aliengo->getFrameAngularVelocity(aliengo->getFrameIdxByName("imu_joint"),imu_simulation.imu_AngularVelocity);
        aliengo->getFrameOrientation(aliengo->getFrameIdxByName("imu_joint"),imu_simulation.imu_Rotation);

        if (controlDecimation == 0)
        {
            imu_simulation.imu_Velocity_bef = imu_simulation.imu_Velocity;
        }

        imu_simulation.imu_data = imu_simulation.imu_simulation(imu_simulation.imu_Rotation, imu_simulation.imu_AngularVelocity, imu_simulation.imu_Velocity, imu_simulation.imu_Velocity_bef, dt, world);

        if (controlDecimation++ % 50 == 0)
        {
//            aliengo->setGeneralizedCoordinate({0, 0, 0.48, 1, 0.0, 0.0, 0.0, 0.0, 0.5, -1,
//                                               0, 0.5, -1, 0.00, 0.5, -1, 0, 0.5, -1});
//            std::cout<<bodyposition<<std::endl;
//            std::cout<<bodyvelocity<<std::endl;
//            std::cout<<bodyangularvelocity<<std::endl;
//            std::cout<<bodyrotation<<std::endl;


        imu_simulation.imu_data_print();


        }

        if (controlDecimation % 50 != 0)
            return;







        /// laikago joint PD controller
        Eigen::VectorXd jointNominalConfig(19), jointVelocityTarget(18);
        jointVelocityTarget.setZero();

        for (size_t i = 0; i < N; i++) {
            for (size_t j = 0; j < N; j++) {
                jointNominalConfig << 0, 0, 0, 0, 0, 0, 0, 0.0, 0.5, -1, 0, 0.5, -1, 0.00, 0.5, -1, 0, 0.5, -1.;

                for (size_t k = 0; k < aliengo->getGeneralizedCoordinateDim(); k++)
                    jointNominalConfig(k) += distribution(generator);

                aliengo->setPdTarget(jointNominalConfig, jointVelocityTarget);
            }
        }
    };

    vis->setControlCallback(controller);

    /// set camera
    vis->select(aliengoVis->at(0));
    vis->getCameraMan()->setYawPitchDist(Ogre::Radian(0.), Ogre::Radian(-1.), 3);

    /// run the app
    vis->run();

    /// terminate
    vis->closeApp();

    return 0;
}
