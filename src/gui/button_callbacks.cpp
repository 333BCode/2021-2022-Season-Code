#include "gui/button_callbacks.hpp"
#include "drivetrain.hpp"

lv_res_t setVirtualBotPos(lv_obj_t* tile) {

    uint32_t tileID = lv_obj_get_free_num(tile);
    Drivetrain::setPosition(tileID % 10 * 144.0 / 6.0, (tileID / 10) * 144.0 / 6.0, 90);

    return LV_RES_OK;

}