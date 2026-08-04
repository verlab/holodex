#pragma once
/**
 * DFRobot_BMI160 includes <arduino.h> (lowercase). On Linux that is a
 * different name from the framework's Arduino.h; on Windows the filesystem
 * is case-insensitive so a naive "#include <Arduino.h>" here recurses forever
 * and never reaches the real header. include_next skips this file.
 */
#include_next <Arduino.h>
