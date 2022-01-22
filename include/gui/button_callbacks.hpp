#ifndef _BUTTON_CALLBACKS_HPP_
#define _BUTTON_CALLBACKS_HPP_

#include "display/lvgl.h"

/**
 * Forward declares callback functions to be assigned to GUI buttons
 */

lv_res_t changeTab(lv_obj_t* tab);
lv_res_t updateAutonTargets(lv_obj_t* target);
lv_res_t setAuton(lv_obj_t* autonSwitch);

#endif