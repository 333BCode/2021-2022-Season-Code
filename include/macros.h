#ifndef _MACROS_H_
#define _MACROS_H_

/**
 * Macros to quickly change functionality of different parts of the code at compile time rather than run time
 */

// When not defined, the screen will clear, except for the auton selector, when the bot is enabled
#define DISPLAY_DEBUG

// When defined, provided DISPLAY_DEBUG is as well,
// the joysticks will move the virtual bot on the brain screen rather than the drivetrain
// #define BRAIN_SCREEN_GAME_MODE

// When defined, driver control will default to make use of autonomous functions for subsystem 3
// #define DEFAULT_TO_MACROS_IN_OPCONTROL

#endif