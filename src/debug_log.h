#pragma once
#include <SettingsGyverWS.h>


#define debug

#ifdef debug
static sets::Logger logger = sets::Logger(1024);
#define DEBUG_PRINT(x) {Serial.print(x); logger.print(x);}
#define DEBUG_PRINTLN(x) {Serial.println(x); logger.println(x);}
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif
