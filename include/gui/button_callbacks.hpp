#ifndef _BUTTON_CALLBACKS_HPP_
#define _BUTTON_CALLBACKS_HPP_

#include "display/lvgl.h"

/**
 * Forward declares callback functions to be assigned to GUI buttons
 */

// updates the displayed tab on the GUI
lv_res_t changeTab(lv_obj_t* tab);
// updates global variables that are used by autons to specify behavior
lv_res_t updateAutonTargets(lv_obj_t* target);
// updates the auton that will be run at the start of the autonomous period
lv_res_t setAuton(lv_obj_t* autonSwitch);

#endif