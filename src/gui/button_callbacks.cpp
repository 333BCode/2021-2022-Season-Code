#include "gui/button_callbacks.hpp"
#include "autonomous.hpp"
#include "gui/display.hpp"
#include "drivetrain.hpp"
#include "pros/rtos.h"

lv_res_t setVirtualBotPos(lv_obj_t* tile) {

    if (!pros::competition::is_autonomous()) {
        uint32_t tileID = lv_obj_get_free_num(tile);
        Drivetrain::setPosition(tileID % 10 * 24.0 + 12.0, (tileID / 10) * 24.0 + 12.0, 90);
    }

    return LV_RES_OK;

}

lv_res_t changeTab(lv_obj_t* tab) {

    uint32_t tabID = lv_obj_get_free_num(tab);
    DisplayControl* displayControl = static_cast<DisplayControl*>(lv_obj_get_free_ptr(tab));

    switch (tabID) {

        case 0:

            lv_btn_set_state(displayControl->odomSwitch, LV_BTN_STATE_TGL_PR);
            lv_btn_set_state(displayControl->autonSelectionSwitch, LV_BTN_STATE_TGL_REL);
            lv_btn_set_state(displayControl->debugSwitch, LV_BTN_STATE_TGL_REL);

            lv_obj_set_hidden(displayControl->odomTab, false);
            lv_obj_set_hidden(displayControl->autonSelectionTab, true);
            lv_obj_set_hidden(displayControl->debugTab, true);

            lv_obj_set_style(displayControl->odomSwitchText, &displayControl->pressedTextStyle);
            lv_obj_set_style(displayControl->autonSelectionSwitchText, &displayControl->positionDataStyle);
            lv_obj_set_style(displayControl->debugSwitchText, &displayControl->positionDataStyle);

            lv_obj_set_hidden(displayControl->field, false);
            lv_obj_align(displayControl->field, NULL, LV_ALIGN_IN_RIGHT_MID, 0, 0);

            break;
        case 1:

            lv_btn_set_state(displayControl->odomSwitch, LV_BTN_STATE_TGL_REL);
            lv_btn_set_state(displayControl->autonSelectionSwitch, LV_BTN_STATE_TGL_PR);
            lv_btn_set_state(displayControl->debugSwitch, LV_BTN_STATE_TGL_REL);

            lv_obj_set_hidden(displayControl->odomTab, true);
            lv_obj_set_hidden(displayControl->autonSelectionTab, false);
            lv_obj_set_hidden(displayControl->debugTab, true);

            lv_obj_set_style(displayControl->odomSwitchText, &displayControl->positionDataStyle);
            lv_obj_set_style(displayControl->autonSelectionSwitchText, &displayControl->pressedTextStyle);
            lv_obj_set_style(displayControl->debugSwitchText, &displayControl->positionDataStyle);

            lv_obj_set_hidden(displayControl->field, false);
            lv_obj_align(displayControl->field, NULL, LV_ALIGN_CENTER, 0, 0);

            break;
        case 2:

            lv_btn_set_state(displayControl->odomSwitch, LV_BTN_STATE_TGL_REL);
            lv_btn_set_state(displayControl->autonSelectionSwitch, LV_BTN_STATE_TGL_REL);
            lv_btn_set_state(displayControl->debugSwitch, LV_BTN_STATE_TGL_PR);

            lv_obj_set_hidden(displayControl->odomTab, true);
            lv_obj_set_hidden(displayControl->autonSelectionTab, true);
            lv_obj_set_hidden(displayControl->debugTab, false);

            lv_obj_set_style(displayControl->odomSwitchText, &displayControl->positionDataStyle);
            lv_obj_set_style(displayControl->autonSelectionSwitchText, &displayControl->positionDataStyle);
            lv_obj_set_style(displayControl->debugSwitchText, &displayControl->pressedTextStyle);

            lv_obj_set_hidden(displayControl->field, true);

            break;

    }

    return LV_RES_OK;

}

lv_res_t setAuton(lv_obj_t* autonSwitch) {

    lv_btn_state_t state = lv_btn_get_state(autonSwitch);
    DisplayControl* displayControl = static_cast<DisplayControl*>(lv_obj_get_free_ptr(autonSwitch));

    if(state == LV_BTN_STATE_REL) {

        lv_btn_set_state(autonSwitch, LV_BTN_STATE_TGL_PR);

        if (displayControl->upperRedAutonSwitch != autonSwitch) {
            lv_btn_set_state(displayControl->upperRedAutonSwitch, LV_BTN_STATE_REL);
        }
        if (displayControl->lowerRedAutonSwitch != autonSwitch) {
            lv_btn_set_state(displayControl->lowerRedAutonSwitch, LV_BTN_STATE_REL);
        }
        if (displayControl->upperBlueAutonSwitch != autonSwitch) {
            lv_btn_set_state(displayControl->upperBlueAutonSwitch, LV_BTN_STATE_REL);
        }
        if (displayControl->lowerBlueAutonSwitch != autonSwitch) {
            lv_btn_set_state(displayControl->lowerBlueAutonSwitch, LV_BTN_STATE_REL);
        }

    autonSelectionMutex.take(TIMEOUT_MAX);

        uint32_t autonNum = lv_obj_get_free_num(autonSwitch);
        if (autonNum == 0) {
            auton = platformUpSide;
        } else if (autonNum == 1) {
            auton = platformDownSide;
        } else {
            auton = awp;
        }

    autonSelectionMutex.give();

    } else {

        lv_btn_set_state(autonSwitch, LV_BTN_STATE_REL);

    autonSelectionMutex.take(TIMEOUT_MAX);
        auton = skills;
    autonSelectionMutex.give();

    }

    return LV_RES_OK;

}