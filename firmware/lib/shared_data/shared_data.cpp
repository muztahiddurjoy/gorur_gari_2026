// SharedData.cpp
#include "shared_data.h"

SharedData shared;
SemaphoreHandle_t sharedMutex = nullptr;   // will be created in setup()