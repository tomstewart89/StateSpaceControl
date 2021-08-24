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

    for (int i = 0; i < 1000; ++i)
    {
        CartPole::loop();
    }

    Serial.buf.str("");

    CartPole::loop();

    EXPECT_STREQ(Serial.buf.str().c_str(), "cart position = 3.50 stick angle = 0.00\n");
}

namespace MotorPosition
{
#include "../examples/MotorPosition/MotorPosition.ino"
}

TEST(Examples, MotorPosition)
{
    MotorPosition::setup();

    for (int i = 0; i < 1000; ++i)
    {
        MotorPosition::loop();
    }

    Serial.buf.str("");

    MotorPosition::loop();

    EXPECT_STREQ(Serial.buf.str().c_str(), "angle = 2.20 velocity = 0.00 current = 0.00\n");
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
