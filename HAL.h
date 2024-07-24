//Hardware Abstraction Layer - To initialise Hardware based on requirements

#ifndef HAL_H
#define HAL_H

#include <stdint.h>

class HAL
{
public:
   void initMotors() ;
   void delayMs(int ms);
   unsigned long currentMicros();
};

#endif
