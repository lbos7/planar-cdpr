#ifndef CDPR_SERIAL_H_
#define CDPR_SERIAL_H_

#include <Arduino.h>
#include "cdpr.hpp"

extern CDPR cdpr;

/**
 * @brief Executes serial commands.
 * 
 * @param cmd The serial command from the user.
 */
void processCommand(String cmd);

#endif  // CDPR_SERIAL_H_