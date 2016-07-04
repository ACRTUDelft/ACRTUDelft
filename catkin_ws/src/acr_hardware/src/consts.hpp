#ifndef Hardware_Consts_H
#define Hardware_Consts_H

/* 
 * Status constants
 * Codings for various status states
 */
#define MODULE_OK 1
#define MODULE_FULL 0
#define MODULE_IDLE 2
#define MODULE_INTERACT 3

/*
 * Settings
 */
 
#define MODULES 3
#define ULTRASONIC_SENSORS 4

#define ULTRASONIC_MIN_RANGE 0.02
#define ULTRASONIC_MAX_RANGE 4.0
#define ULTRASONIC_FIELD_OF_VIEW 0.523598776	// 30 degrees in radians

#endif
