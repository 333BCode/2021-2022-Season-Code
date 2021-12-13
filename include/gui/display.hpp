#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include "display/lvgl.h"
#include "api.h"
#include "macros.h"

/**
 * Declares the DisplayControl class
 *
 * When constructed, the brain screen will be initialized
 * When destructed, the brain screen will be deleted
 */

class DisplayControl final {
public:

    // initialize brain screen
    DisplayControl();
    // disallow copy construction
    DisplayControl(const DisplayControl&) = delete;
    // delete brain screen
    ~DisplayControl();
    // disallow copy assignment
    void operator=(const DisplayControl&) = delete;

#ifndef DISPLAY_DEBUG
    // Deletes all of the screen except the auton selector to save on resources
    void cleanScreen();
#endif

    // Moves the virtual bot on the brain screen
    // Updates the data displayed on the brain screen if updateValues
    void updateOdomData(bool updateValues);
    // Allows changeTab to show / hide lvgl objects
    friend lv_res_t changeTab(lv_obj_t* tab);
    // Allows setAuton to properly update the auton buttons
    friend lv_res_t setAuton(lv_obj_t* autonSwitch);

private:

    /**
     * Objects
     */

    lv_obj_t* tabSpace;
    lv_obj_t* tabSwitcher;
    int       tabBarHeight;

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
    lv_point_t directionIndicatorEndpoints[2];
    lv_obj_t* positionData;

    lv_obj_t* autonSelectionTab;
    lv_obj_t* upperRedAutonSwitch;
    lv_obj_t* lowerRedAutonSwitch;
    lv_obj_t* upperBlueAutonSwitch;
    lv_obj_t* lowerBlueAutonSwitch;

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

    lv_style_t autonBtnPr;
    lv_style_t autonBtnRel;

};

#endif