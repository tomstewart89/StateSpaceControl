#include <StateSpaceControl.h>

void setup() 
{
    StateSpaceController<4,1,2> s;

    float M = 0.5, m = 0.2, b = 0.1, l = 0.3, I = 0.006, g = 9.81;
    float  C = (I * (M + m) + M * m * l * l);

    float F[][4] = {{0, 1, 0, 0},
                    {0, -(I + m * l * l) * b / C, m * m * g * l * l / C, 0},
                    {0, 0, 0, 1},
                    {0, -m * l * b / C, m * g * l * (M + m) / C, 0}};

    float G[][1] = {{0},
                    {(I + m * l * l) / C},
                    {0},
                    {(m * l) / C}};

    float H[][4] = {{1, 0, 0, 0},
                    {0, 0, 1, 0}};

    float K[][4] = {-70.7107, -37.8345, 105.5298, 20.9238};

    float L[][2] = {{8.26, -0.1}, {169.92, -4.02},
                    {-0.14, 8.32},{-76.2, 176.04}};

    s.systemMatrix = F;
    s.inputMatrix = G;
    s.outputMatrix = H;

    s.controlGain = K;
    s.estimatorGain = L;

    s.Initialise();

    Matrix<4> x;
    Matrix<2> y;

    x.Fill(0);

    x(0) = 1;
    x(1) = 2;
    x(2) = 3;
    x(3) = 4;

    s.referenceInput(0,0) = 3;
    s.referenceInput(0,1) = 0.0;

    for(int i = 0; i < 1000; i++)
    {   
        x += (s.systemMatrix * x + s.inputMatrix * s.controlInput) * 0.01;
        y = s.outputMatrix * x;

        s.Update(y, 0.01);

        Serial << s.stateEstimate << '\n';
    }
}

void loop() 
{
  // Iterate the feedback loop
	s.Update(sensorRead, 0.05);
}

