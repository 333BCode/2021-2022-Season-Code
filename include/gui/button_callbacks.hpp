#ifndef BUTTON_CALLBACKS_HPP
#define BUTTON_CALLBACKS_HPP

#include "display/lvgl.h"

lv_res_t setVirtualBotPos(lv_obj_t* tile);
lv_res_t changeTab(lv_obj_t* tab);
lv_res_t setAuton(lv_obj_t* autonSwitch);

#endif