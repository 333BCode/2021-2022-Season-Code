#include "gui/display.hpp"
#include "gui/button_callbacks.hpp"
#include "pros/rtos.h"
#include "util/conversions.hpp"
#include "drivetrain.hpp"
#include "macros.h"

#include <string>

constexpr short defaultScreen = 1;

static lv_obj_t * newButton(lv_obj_t* parent, lv_coord_t xPos, lv_coord_t yPos, lv_coord_t width, lv_coord_t height,
    uint32_t freeNum, lv_res_t (*action)(lv_obj_t*), bool toggle)
{
    
    lv_obj_t* newBtn = lv_btn_create(parent, NULL);

    lv_obj_set_pos(newBtn, xPos, yPos);
    lv_obj_set_size(newBtn, width, height);
    lv_obj_set_free_num(newBtn, freeNum);
    if (action) {
        lv_btn_set_action(newBtn, LV_BTN_ACTION_CLICK, action);
    }
    lv_btn_set_toggle(newBtn, toggle);

    return newBtn;

}

static void setupStyle(lv_style_t* style, lv_style_t* copy, lv_color_t bodyColor,
    int16_t borderWidth, lv_color_t borderColor)
{

    lv_style_copy(style, copy);

    style->body.main_color = bodyColor;
    style->body.grad_color = bodyColor;

    style->body.border.width = borderWidth;
    style->body.radius = 0;
    style->body.border.color = borderColor;

    style->body.padding.inner = 0;
    style->body.padding.hor = 0;
    style->body.padding.ver = 0;

}

static void setupStyle(lv_style_t* style, lv_style_t* copy, lv_color_t bodyColor){

    lv_style_copy(style, copy);

    style->body.main_color = bodyColor;
    style->body.grad_color = bodyColor;
    style->body.border.width = 0;
    style->body.radius = 0;

    style->body.padding.inner = 0;
    style->body.padding.hor = 0;
    style->body.padding.ver = 0;

}

lv_res_t doNothing(lv_obj_t*) {
    return LV_RES_INV;
}

DisplayControl::DisplayControl()

    : tabSpace {lv_obj_create(lv_scr_act(), NULL)}, tabSwitcher {lv_obj_create(lv_scr_act(), NULL)},

    odomTab {lv_obj_create(tabSpace, NULL)},
    positionData {lv_label_create(odomTab, NULL)},

    autonSelectionTab {lv_obj_create(tabSpace, NULL)},

    debugTab {lv_obj_create(tabSpace, NULL)},

    field {lv_obj_create(tabSpace, NULL)},
    redPlatform {lv_obj_create(field, NULL)},
    bluePlatform {lv_obj_create(field, NULL)}

{

    /**
     * Styles
     */

    lv_style_copy(&tabviewStyleIndic, &lv_style_plain);
    tabviewStyleIndic.body.padding.inner = 5;
    setupStyle(&tabviewStyle, &lv_style_plain_color, LV_COLOR_NAVY);
    setupStyle(&tabviewBtnPressedStyle, &lv_style_plain_color, LV_COLOR_CYAN);

    setupStyle(&odomPageStyle, &lv_style_plain_color, LV_COLOR_BLACK);
    setupStyle(&fieldStyle, &lv_style_plain_color, LV_COLOR_GRAY, 1, LV_COLOR_WHITE);
    
    setupStyle(&virtualBotStyle, &lv_style_plain, LV_COLOR_MAGENTA);
    virtualBotStyle.body.radius = LV_RADIUS_CIRCLE;
    lv_style_copy(&virtualBotDirectionIndicatorStyle, &lv_style_plain);
    virtualBotDirectionIndicatorStyle.line.width = 5;
    virtualBotDirectionIndicatorStyle.line.opa = LV_OPA_100;
    virtualBotDirectionIndicatorStyle.line.color = LV_COLOR_AQUA;

    lv_style_copy(&positionDataStyle, &lv_style_plain);
    positionDataStyle.text.color = LV_COLOR_WHITE;
    positionDataStyle.text.opa = LV_OPA_100;

    lv_style_copy(&pressedTextStyle, &positionDataStyle);
    pressedTextStyle.text.color = LV_COLOR_BLACK;

    setupStyle(&redPlatformStyle, &lv_style_plain_color, LV_COLOR_RED);
    setupStyle(&bluePlatformStyle, &lv_style_plain_color, LV_COLOR_BLUE);

    setupStyle(&autonBtnPr, &lv_style_plain_color, LV_COLOR_SILVER, 5, LV_COLOR_MAGENTA);
    lv_style_copy(&autonBtnRel, &autonBtnPr);
    autonBtnRel.body.border.color = LV_COLOR_WHITE;


    /*
     * Set up tab system
     */

    lv_obj_set_size(tabSwitcher, lv_obj_get_width(lv_scr_act()), tabBarHeight);
    lv_obj_set_style(tabSwitcher, &tabviewStyleIndic);

    odomSwitch = newButton(tabSwitcher, 0, 0, lv_obj_get_width(tabSwitcher) / 3.1, tabBarHeight - 5, 
        0, changeTab, true);
    lv_obj_align(odomSwitch, NULL, LV_ALIGN_IN_LEFT_MID, 4, 0);
    lv_obj_set_free_ptr(odomSwitch, this);
    lv_btn_set_style(odomSwitch, LV_BTN_STYLE_TGL_PR, &tabviewBtnPressedStyle);
    lv_btn_set_style(odomSwitch, LV_BTN_STYLE_TGL_REL, &tabviewStyle);

    odomSwitchText = lv_label_create(odomSwitch, NULL);
    lv_obj_align(odomSwitchText, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(odomSwitchText, "Odom");

    autonSelectionSwitch = newButton(tabSwitcher, 0, 0, lv_obj_get_width(tabSwitcher) / 3.1, tabBarHeight - 5, 
        1, changeTab, true);
    lv_obj_align(autonSelectionSwitch, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_free_ptr(autonSelectionSwitch, this);
    lv_btn_set_style(autonSelectionSwitch, LV_BTN_STYLE_TGL_PR, &tabviewBtnPressedStyle);
    lv_btn_set_style(autonSelectionSwitch, LV_BTN_STYLE_TGL_REL, &tabviewStyle);

    autonSelectionSwitchText = lv_label_create(autonSelectionSwitch, NULL);
    lv_obj_align(autonSelectionSwitchText, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(autonSelectionSwitchText, "Auton");

    debugSwitch = newButton(tabSwitcher, 0, 0, lv_obj_get_width(tabSwitcher) / 3.1, tabBarHeight - 5, 
        2, changeTab, true);
    lv_obj_align(debugSwitch, NULL, LV_ALIGN_IN_RIGHT_MID, -4, 0);
    lv_obj_set_free_ptr(debugSwitch, this);
    lv_btn_set_style(debugSwitch, LV_BTN_STYLE_TGL_PR, &tabviewBtnPressedStyle);
    lv_btn_set_style(debugSwitch, LV_BTN_STYLE_TGL_REL, &tabviewStyle);

    debugSwitchText = lv_label_create(debugSwitch, NULL);
    lv_obj_align(debugSwitchText, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(debugSwitchText, "Debug");

    lv_obj_set_size(tabSpace, lv_obj_get_width(lv_scr_act()), lv_obj_get_height(lv_scr_act()) - tabBarHeight);
    lv_obj_set_pos(tabSpace, 0, tabBarHeight);

    tileLength = lv_obj_get_height(tabSpace) / 6.0;

    /**
     * Odom Tab
     */
    
    lv_obj_set_size(odomTab, lv_obj_get_width(tabSpace), lv_obj_get_height(tabSpace));
    lv_obj_set_style(odomTab, &odomPageStyle);

    // Position Data
    lv_obj_align(positionData, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 20);
    lv_label_set_text(positionData, "Odom starting up...");
    lv_obj_set_style(positionData, &positionDataStyle);

    /*
     * Auton Selection Tab
     */
    
    lv_obj_set_size(autonSelectionTab, lv_obj_get_width(tabSpace), lv_obj_get_height(tabSpace));
    lv_obj_set_style(autonSelectionTab, &odomPageStyle);

    upperRedAutonSwitch = newButton(autonSelectionTab, 0, 0, tileLength * 2, tileLength * 2, 1, setAuton, false);
    lv_obj_align(upperRedAutonSwitch, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10);
    lv_obj_set_free_ptr(upperRedAutonSwitch, this);
    lv_btn_set_style(upperRedAutonSwitch, LV_BTN_STYLE_TGL_PR, &autonBtnPr);
    lv_btn_set_style(upperRedAutonSwitch, LV_BTN_STYLE_PR, &autonBtnPr);
    lv_btn_set_style(upperRedAutonSwitch, LV_BTN_STYLE_REL, &autonBtnRel);

    lowerRedAutonSwitch = newButton(autonSelectionTab, 0, 0, tileLength * 2, tileLength * 2, 0, setAuton, false);
    lv_obj_align(lowerRedAutonSwitch, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 10, -10);
    lv_obj_set_free_ptr(lowerRedAutonSwitch, this);
    lv_btn_set_style(lowerRedAutonSwitch, LV_BTN_STYLE_TGL_PR, &autonBtnPr);
    lv_btn_set_style(lowerRedAutonSwitch, LV_BTN_STYLE_PR, &autonBtnPr);
    lv_btn_set_style(lowerRedAutonSwitch, LV_BTN_STYLE_REL, &autonBtnRel);

    upperBlueAutonSwitch = newButton(autonSelectionTab, 0, 0, tileLength * 2, tileLength * 2, 0, setAuton, false);
    lv_obj_align(upperBlueAutonSwitch, NULL, LV_ALIGN_IN_TOP_RIGHT, -10, 10);
    lv_obj_set_free_ptr(upperBlueAutonSwitch, this);
    lv_btn_set_style(upperBlueAutonSwitch, LV_BTN_STYLE_TGL_PR, &autonBtnPr);
    lv_btn_set_style(upperBlueAutonSwitch, LV_BTN_STYLE_PR, &autonBtnPr);
    lv_btn_set_style(upperBlueAutonSwitch, LV_BTN_STYLE_REL, &autonBtnRel);

    lowerBlueAutonSwitch = newButton(autonSelectionTab, 0, 0, tileLength * 2, tileLength * 2, 1, setAuton, false);
    lv_obj_align(lowerBlueAutonSwitch, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_free_ptr(lowerBlueAutonSwitch, this);
    lv_btn_set_style(lowerBlueAutonSwitch, LV_BTN_STYLE_TGL_PR, &autonBtnPr);
    lv_btn_set_style(lowerBlueAutonSwitch, LV_BTN_STYLE_PR, &autonBtnPr);
    lv_btn_set_style(lowerBlueAutonSwitch, LV_BTN_STYLE_REL, &autonBtnRel);

    /*
     * Debug Tab
     */
    
    lv_obj_set_size(debugTab, lv_obj_get_width(tabSpace), lv_obj_get_height(tabSpace));
    lv_obj_set_style(debugTab, &odomPageStyle);

    /*
     * Field
     */

    lv_obj_set_size(field, lv_obj_get_height(tabSpace), lv_obj_get_height(tabSpace));
    lv_obj_set_style(field, &fieldStyle);

    // Field Tiles

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {

            // platform tiles
            if ((j == 0 || j == 5) && (i == 2 || i == 3)) {
                continue;
            }

            lv_obj_t* newTile = newButton(field, tileLength * j, tileLength * i, tileLength, tileLength,
                10 * (5 - i) + j, setVirtualBotPos, false);
            lv_btn_set_style(newTile, LV_BTN_STYLE_PR, &fieldStyle);
            lv_btn_set_style(newTile, LV_BTN_STYLE_REL, &fieldStyle);
        
        }
    }

    lv_obj_set_size(redPlatform, tileLength, 2 * tileLength);
    lv_obj_set_pos(redPlatform, 0, 2 * tileLength);
    lv_obj_set_style(redPlatform, &redPlatformStyle);

    lv_obj_set_size(bluePlatform, tileLength, 2 * tileLength);
    lv_obj_set_pos(bluePlatform, 5 * tileLength, 2 * tileLength);
    lv_obj_set_style(bluePlatform, &bluePlatformStyle);

    // Virtual Bot
    virtualBot = lv_led_create(field, NULL);
    lv_led_on(virtualBot);
    virtualBotDirectionIndicator = lv_line_create(field, NULL);

    lv_obj_set_size(virtualBot, tileLength * 0.75, tileLength * 0.75);
    lv_obj_set_style(virtualBot, &virtualBotStyle);

    lv_obj_set_style(virtualBotDirectionIndicator, &virtualBotDirectionIndicatorStyle);

    /*
     * show default screen
     */
    
    switch (defaultScreen) {
        
        case 0:
            changeTab(odomSwitch);
            break;
        case 1:
            changeTab(autonSelectionSwitch);
            break;
        default:
            changeTab(debugSwitch);
            break;

    }

}

DisplayControl::~DisplayControl() {
    lv_obj_clean(lv_scr_act());
}

#ifndef DISPLAY_DEBUG
void DisplayControl::cleanScreen() {

    changeTab(autonSelectionSwitch);
    lv_obj_clean(tabSwitcher);
    lv_obj_del(odomTab);
    lv_obj_del(debugTab);
    lv_obj_del(field);

}
#endif

void DisplayControl::updateOdomData(bool updateValues) {

    Drivetrain::XYHPoint odomData = Drivetrain::getPosition();

    if (updateValues) {
        
        std::string newOdomReadout = "Values in inches\nand degrees:\n\n"
            "X Position: " + std::to_string(round(odomData.x        * 1000) / 1000).substr(0, 6) + "\n"
            "Y Position: " + std::to_string(round(odomData.y        * 1000) / 1000).substr(0, 6) + "\n"
            "Heading:    " + std::to_string(round(odomData.heading  * 1000) / 1000).substr(0, 6);

        lv_label_set_text(positionData, newOdomReadout.c_str());

    }

    odomData.heading = conversions::radians(odomData.heading);

    // divide by 24 (144 / 6) instead of 144 since 6 * tile length = field length
    lv_coord_t x = (odomData.x / 24) * tileLength - 12;
    lv_coord_t y = 186 - (odomData.y / 24) * tileLength;

    lv_obj_set_pos(virtualBot, x, y);
    
    directionIndicatorEndpoints[0] = {static_cast<lv_coord_t>(x + 12), static_cast<lv_coord_t>(y + 12)};
    directionIndicatorEndpoints[1] = {
        static_cast<lv_coord_t>(x + 12 + tileLength * cos(odomData.heading) * 0.75),
        static_cast<lv_coord_t>(y + 12 - tileLength * sin(odomData.heading) * 0.75)
    };

    lv_line_set_points(virtualBotDirectionIndicator, directionIndicatorEndpoints, 2);

}