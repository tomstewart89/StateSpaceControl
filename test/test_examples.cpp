#include <BasicLinearAlgebra.h>
#include <StateSpaceControl.h>
#include <gtest/gtest.h>

using namespace BLA;

namespace CartPole
{
#include "../examples/CartPole/CartPole.ino"
}

TEST(Examples, CartPole)
{
    CartPole::setup();

    for (int i = 0; i < 100; ++i)
    {
        CartPole::loop();
    }

    Serial.begin(0);

    CartPole::loop();

    std::cout << Serial.buf.str() << "\n";

    // EXPECT_STREQ(
    //     Serial.buf.str().c_str(),
    //     "distance from drone to ground station: 1.58\n"
    //     "drone pose: R: [[0.00,1.00,0.00],[-1.00,0.00,0.00],[0.00,0.00,1.00]] p: [[1.50],[0.00],[0.50]]\n"
    //     "other_drone_relative_to_GS: R: [[0.00,0.97,-0.26],[-1.00,0.00,0.00],[0.00,0.26,0.97]] p: "
    //     "[[3.50],[0.00],[0.50]]\n"
    //     "T_GS_otherdrone: R: [[0.00,0.97,-0.26],[-1.00,0.00,0.00],[0.00,0.26,0.97]] p: [[3.50],[0.00],[0.50]]\n"
    //     "T_otherdrone_GS: R: [[0.00,-1.00,0.00],[0.97,0.00,0.26],[-0.26,0.00,0.97]] p: [[-0.00],[-3.51],[0.42]]");
}

namespace MotorPosition
{
#include "../examples/MotorPosition/MotorPosition.ino"
}

TEST(Examples, MotorPosition)
{
    MotorPosition::setup();

    for (int i = 0; i < 100; ++i)
    {
        MotorPosition::loop();
    }

    Serial.begin(0);

    MotorPosition::loop();

    std::cout << Serial.buf.str() << "\n";

    // EXPECT_STREQ(Serial.buf.str().c_str(),
    //              "[[-1.00,0.00,-0.00],[0.00,1.00,-0.00],[-0.00,-0.00,-1.00]]\n"
    //              "[[1.00,0.00,-0.00],[0.00,1.00,-0.00],[0.00,0.00,1.00]]\n"
    //              "Gimbal locked: [[0.00,1.00,-0.00],[-0.00,0.00,1.00],[1.00,0.00,0.00]]\n"
    //              "Still gimbal locked: [[-0.00,0.00,-1.00],[-0.00,1.00,0.00],[1.00,0.00,-0.00]]\n"
    //              "Tricky rotation in euler angles[[1.57],[-0.79],[3.14]]");
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
