#ifndef Consts_H
#define Consts_H

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
#define ULTRASONIC_MIN_DIST 50
#define MOVE_SPEED 1
#define TURN_SPEED 1

#endif
