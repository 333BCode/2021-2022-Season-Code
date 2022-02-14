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

    // struct to specify lvgl button construction for the auton selector
    struct Auton {

        using auton_t = void(*)();

        // constructor
        Auton(auton_t autonFunc, const char* name, bool showElements = false);

        // function pointer to the auton function that should be called if this button is selected
        auton_t autonFunc;
        // displayed name of the auton button
        const char* name;
        // determines if the additional auton target buttons will be displayed
        bool showElements;

    };

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
    // Allows updateAutonTargets to update the auton target buttons
    friend lv_res_t updateAutonTargets(lv_obj_t *target);
    // Allows setAuton to properly update the auton buttons, selection indicator,
    // and set auton to point to the proper auton function
    friend lv_res_t setAuton(lv_obj_t* autonSwitch);

private:

    /**
     * arrays of Autons to specify creation of auton selection buttons on the gui
     */ 
    static const Auton upperAutons[3];
    static const Auton lowerAutons[2];

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

    lv_obj_t* odomTab;
    double tileLength;
    lv_obj_t* virtualBot;
    lv_obj_t* virtualBotDirectionIndicator;
    lv_point_t directionIndicatorEndpoints[2];
    lv_obj_t* positionData;

    lv_obj_t* autonSelectionTab;
    lv_obj_t* selectedIndicator;
    // keeps track of which auton button is selected
    uint32_t selectedNum {500};

    lv_obj_t* field;
    lv_obj_t* redPlatform;
    lv_obj_t* bluePlatform;
    lv_obj_t* tallNeutralMogo;
    lv_obj_t* shortNeutralMogo;
    lv_obj_t* rings;

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

    lv_style_t autonBtnStyle;
    lv_style_t autonBtnNameStyle;

    lv_style_t selectedIndicatorStyle;

    lv_style_t mogoSelectedStyle;
    lv_style_t mogoStyle;
    lv_style_t ringSelectedStyle;
    lv_style_t ringStyle;

};

#endif