#ifndef DISPLAY_H
#define DISPLAY_H

#include "display/lvgl.h"

class DisplayControl final {
public:

    DisplayControl();
    ~DisplayControl();

    void updateOdomData(bool updateValues);
    friend lv_res_t changeTab(lv_obj_t* tab);

private:

    lv_obj_t* tabSpace;
    lv_obj_t* tabSwitcher;
    int       tabBarHeight {40};

    lv_obj_t* odomSwitch;
    lv_obj_t* odomSwitchText;
    lv_obj_t* autonSelectionSwitch;
    lv_obj_t* autonSelectionSwitchText;
    lv_obj_t* debugSwitch;
    lv_obj_t* debugSwitchText;

    lv_obj_t* odomTab;
    double tileLength;
    lv_obj_t* virtualBot;
    lv_obj_t* virtualBotDirectionIndicator;
    lv_obj_t* positionData;

    lv_obj_t* autonSelectionTab;

    lv_obj_t* debugTab;

    lv_obj_t* field;
    lv_obj_t* redPlatform;
    lv_obj_t* bluePlatform;

    /**
     * Styles
     */

    lv_style_t tabviewStyleIndic;
    lv_style_t tabviewStyle;
    lv_style_t tabviewBtnPressedStyle;

    lv_style_t odomPageStyle;
    lv_style_t fieldStyle;
    lv_style_t virtualBotStyle;
    lv_style_t virtualBotDirectionIndicatorStyle;

    lv_style_t positionDataStyle;
    lv_style_t pressedTextStyle;

    lv_style_t redPlatformStyle;
    lv_style_t bluePlatformStyle;

};

extern DisplayControl displayControl;

#endif