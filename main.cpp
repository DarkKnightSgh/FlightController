#include "Controller.h"
#include "HAL.cpp"

HAL hal;
QuadcopterController quadcopter(hal);
Communication communication;

int main()
{
    //if stabilisation then
    quadcopter.setup();

    while (true)
    {
        quadcopter.loop(); //stabilisation
    }
    //else
    //Communication communication;


    return 0;
}
