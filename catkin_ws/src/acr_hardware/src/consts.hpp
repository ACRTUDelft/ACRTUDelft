#ifndef Hardware_Consts_H
#define Hardware_Consts_H

/* 
 * Identifying constants
 * Identify parts by their id using a readable name
 */
#define MODULE1 0
#define MODULE2 1
#define MODULE3 2

#define U_LEFT 0
#define U_FRONT_TOP 1
#define U_FRONT_BOTTOM 2
#define U_RIGHT 3

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
