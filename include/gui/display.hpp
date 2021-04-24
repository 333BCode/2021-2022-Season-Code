#ifndef DISPLAY_H
#define DISPLAY_H

#include "display/lvgl.h"
#include "gui/button_callbacks.hpp"

class DisplayControl final {
public:

    DisplayControl();
    ~DisplayControl();

    void updateOdomData();

private:

    /**
     * Objects
     */

    lv_obj_t* tabview;

    lv_obj_t* odomTab;
    lv_obj_t* odomPage;
    lv_obj_t* field;
    lv_obj_t* virtualBot;
    lv_obj_t* virtualBotDirectionIndicator;
    lv_obj_t* positionData;

    lv_obj_t* autonSelectionTab;
    lv_obj_t* autonSelectionPage;

    double tileLength;

    /**
     * Styles
     */

    lv_style_t tabviewStyle;
    lv_style_t tabviewBtnPressedStyle;

    lv_style_t odomPageStyle;
    lv_style_t fieldStyle;
    lv_style_t virtualBotStyle;
    lv_style_t virtualBotDirectionIndicatorStyle;

    lv_style_t positionDataStyle;

};

extern DisplayControl displayControl;

#endif