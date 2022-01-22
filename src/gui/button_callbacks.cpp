#include "gui/button_callbacks.hpp"
#include "autonomous.hpp"
#include "gui/display.hpp"
#include "drivetrain.hpp"
#include "pros/rtos.h"

lv_res_t changeTab(lv_obj_t* tab) {

    uint32_t tabID = lv_obj_get_free_num(tab);
    DisplayControl* displayControl = static_cast<DisplayControl*>(lv_obj_get_free_ptr(tab));

    lv_obj_t* newSwitch;
    lv_obj_t* newTab;
    lv_obj_t* newText;

    lv_obj_t* oldSwitch;
    lv_obj_t* oldTab;
    lv_obj_t* oldText;

    if (tabID) {

        newSwitch = displayControl->autonSelectionSwitch;
        newTab = displayControl->autonSelectionTab;
        newText = displayControl->autonSelectionSwitchText;

        oldSwitch = displayControl->odomSwitch;
        oldTab = displayControl->odomTab;
        oldText = displayControl->odomSwitchText;

    } else {

        newSwitch = displayControl->odomSwitch;
        newTab = displayControl->odomTab;
        newText = displayControl->odomSwitchText;

        oldSwitch = displayControl->autonSelectionSwitch;
        oldTab = displayControl->autonSelectionTab;
        oldText = displayControl->autonSelectionSwitchText;

    }

    lv_btn_set_state(newSwitch, LV_BTN_STATE_TGL_PR);
    lv_btn_set_state(oldSwitch, LV_BTN_STATE_TGL_REL);

    lv_obj_set_hidden(newTab, false);
    lv_obj_set_hidden(oldTab, true);

    lv_obj_set_style(newText, &displayControl->pressedTextStyle);
    lv_obj_set_style(oldText, &displayControl->positionDataStyle);

    lv_obj_set_hidden(displayControl->virtualBot, tabID);
    lv_obj_set_hidden(displayControl->virtualBotDirectionIndicator, tabID);

    return LV_RES_OK;

}

lv_res_t updateAutonTargets(lv_obj_t* target) {
    
    uint32_t freeNum = lv_obj_get_free_num(target);
    DisplayControl* displayControl = static_cast<DisplayControl*>(lv_obj_get_free_ptr(target));

    switch (freeNum) {

        case 0:

            targetTallNeutralMogo = !targetTallNeutralMogo;
            if (targetTallNeutralMogo) {
                lv_btn_set_style(target, LV_BTN_STYLE_PR, &displayControl->mogoSelectedStyle);
                lv_btn_set_style(target, LV_BTN_STYLE_REL, &displayControl->mogoSelectedStyle);
            } else {
                lv_btn_set_style(target, LV_BTN_STYLE_PR, &displayControl->mogoStyle);
                lv_btn_set_style(target, LV_BTN_STYLE_REL, &displayControl->mogoStyle);
            }

        break;
        case 1:
        
            targetShortNeutralMogo = !targetShortNeutralMogo;
            if (targetShortNeutralMogo) {
                lv_btn_set_style(target, LV_BTN_STYLE_PR, &displayControl->mogoSelectedStyle);
                lv_btn_set_style(target, LV_BTN_STYLE_REL, &displayControl->mogoSelectedStyle);
            } else {
                lv_btn_set_style(target, LV_BTN_STYLE_PR, &displayControl->mogoStyle);
                lv_btn_set_style(target, LV_BTN_STYLE_REL, &displayControl->mogoStyle);
            }
        
        break;
        default:
        
            targetRings = !targetRings;
            if (targetRings) {
                lv_btn_set_style(target, LV_BTN_STYLE_PR, &displayControl->ringSelectedStyle);
                lv_btn_set_style(target, LV_BTN_STYLE_REL, &displayControl->ringSelectedStyle);
            } else {
                lv_btn_set_style(target, LV_BTN_STYLE_PR, &displayControl->ringStyle);
                lv_btn_set_style(target, LV_BTN_STYLE_REL, &displayControl->ringStyle);
            }
        
        break;

    }
    
    return LV_RES_OK;

}

lv_res_t setAuton(lv_obj_t* autonSwitch) {

    uint32_t freeNum = lv_obj_get_free_num(autonSwitch);
    DisplayControl* displayControl = static_cast<DisplayControl*>(lv_obj_get_free_ptr(autonSwitch));

    bool hideFieldElements = true;

    if (displayControl->selectedNum == freeNum) {
        
        displayControl->selectedNum = 500;
        lv_obj_set_hidden(displayControl->selectedIndicator, true);

        auton = none;

    } else {

        displayControl->selectedNum = freeNum;
        lv_obj_set_hidden(displayControl->selectedIndicator, false);
        lv_obj_align(displayControl->selectedIndicator, autonSwitch, LV_ALIGN_CENTER, 0, 1);

        const Auton* autonButton;

        if (freeNum < 100) {
            autonButton = &displayControl->upperAutons[freeNum];
        } else {
            autonButton = &displayControl->lowerAutons[freeNum - 100];
        }

        auton = autonButton->autonFunc;
        hideFieldElements = !autonButton->showElements;

    }

    lv_obj_set_hidden(displayControl->tallNeutralMogo, hideFieldElements);
    lv_obj_set_hidden(displayControl->shortNeutralMogo, hideFieldElements);
    lv_obj_set_hidden(displayControl->rings, hideFieldElements);

    return LV_RES_OK;

}