#include "gui/display.hpp"
#include "gui/button_callbacks.hpp"
#include "pros/rtos.h"
#include "util/conversions.hpp"
#include "drivetrain.hpp"
#include "macros.h"

#include <string>

constexpr short defaultScreen = 1;

DisplayControl::Auton::Auton(auton_t autonFunc, const char* name, bool showElements)
    : autonFunc {autonFunc}, name {name}, showElements {showElements} {}

static lv_obj_t* newButton(lv_obj_t* parent, lv_coord_t xPos, lv_coord_t yPos, lv_coord_t width, lv_coord_t height,
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

DisplayControl::DisplayControl()

    : tabSpace {lv_obj_create(lv_scr_act(), NULL)}, tabSwitcher {lv_obj_create(lv_scr_act(), NULL)},

    tabBarHeight {40},

    odomTab {lv_obj_create(tabSpace, NULL)},
    positionData {lv_label_create(odomTab, NULL)},

    autonSelectionTab {lv_obj_create(tabSpace, NULL)},
    selectedIndicator {lv_obj_create(autonSelectionTab, NULL)},

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

    setupStyle(&autonBtnStyle, &lv_style_plain_color, LV_COLOR_SILVER);
    autonBtnStyle.body.radius = 3;

    lv_style_copy(&autonBtnNameStyle, &lv_style_plain);
    autonBtnNameStyle.text.color = LV_COLOR_MAGENTA;
    autonBtnNameStyle.text.opa = LV_OPA_100;

    setupStyle(&selectedIndicatorStyle, &lv_style_plain_color, LV_COLOR_TRANSP, 4, LV_COLOR_BLUE);
    selectedIndicatorStyle.body.radius = 5;

    setupStyle(&mogoSelectedStyle, &lv_style_plain, LV_COLOR_YELLOW);
    mogoSelectedStyle.body.radius = LV_RADIUS_CIRCLE;

    setupStyle(&mogoStyle, &lv_style_plain, LV_COLOR_BLACK);
    mogoStyle.body.radius = LV_RADIUS_CIRCLE;

    setupStyle(&ringSelectedStyle, &lv_style_plain_color, LV_COLOR_PURPLE);
    setupStyle(&ringStyle, &lv_style_plain_color, LV_COLOR_BLACK);

    /*
     * Set up tab system
     */

    lv_obj_set_size(tabSwitcher, lv_obj_get_width(lv_scr_act()), tabBarHeight);
    lv_obj_set_style(tabSwitcher, &tabviewStyleIndic);

    odomSwitch = newButton(tabSwitcher, 0, 0, lv_obj_get_width(tabSwitcher) / 2.05, tabBarHeight - 5, 
        0, changeTab, true);
    lv_obj_align(odomSwitch, NULL, LV_ALIGN_IN_LEFT_MID, 4, 0);
    lv_obj_set_free_ptr(odomSwitch, this);
    lv_btn_set_style(odomSwitch, LV_BTN_STYLE_TGL_PR, &tabviewBtnPressedStyle);
    lv_btn_set_style(odomSwitch, LV_BTN_STYLE_TGL_REL, &tabviewStyle);

    odomSwitchText = lv_label_create(odomSwitch, NULL);
    lv_obj_align(odomSwitchText, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(odomSwitchText, "Odom");

    autonSelectionSwitch = newButton(tabSwitcher, 0, 0, lv_obj_get_width(tabSwitcher) / 2.05, tabBarHeight - 5, 
        1, changeTab, true);
    lv_obj_align(autonSelectionSwitch, NULL, LV_ALIGN_IN_RIGHT_MID, -4, 0);
    lv_obj_set_free_ptr(autonSelectionSwitch, this);
    lv_btn_set_style(autonSelectionSwitch, LV_BTN_STYLE_TGL_PR, &tabviewBtnPressedStyle);
    lv_btn_set_style(autonSelectionSwitch, LV_BTN_STYLE_TGL_REL, &tabviewStyle);

    autonSelectionSwitchText = lv_label_create(autonSelectionSwitch, NULL);
    lv_obj_align(autonSelectionSwitchText, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(autonSelectionSwitchText, "Auton");

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

    for (size_t i = 0; i < std::size(upperAutons); ++i) {

        lv_obj_t* obj = newButton(autonSelectionTab, 0, 0, 1.5 * tileLength, tileLength, i, setAuton, false);
        lv_obj_align(obj, NULL, LV_ALIGN_IN_TOP_LEFT, (0.25 + 2 * i) * tileLength, 0.25 * tileLength);
        lv_obj_set_free_ptr(obj, this);

        lv_btn_set_style(obj, LV_BTN_STYLE_PR, &autonBtnStyle);
        lv_btn_set_style(obj, LV_BTN_STYLE_REL, &autonBtnStyle);

        obj = lv_label_create(obj, NULL);
        lv_label_set_text(obj, upperAutons[i].name);
        lv_obj_set_style(obj, &autonBtnNameStyle);
    
    }

    for (size_t i = 0; i < std::size(lowerAutons); ++i) {

        lv_obj_t* obj = newButton(autonSelectionTab, 0, 0, 1.5 * tileLength, tileLength, 100 + i, setAuton, false);
        lv_obj_align(obj, NULL, LV_ALIGN_IN_LEFT_MID, (0.25 + 2 * i) * tileLength, 0.25 * tileLength);
        lv_obj_set_free_ptr(obj, this);

        lv_btn_set_style(obj, LV_BTN_STYLE_PR, &autonBtnStyle);
        lv_btn_set_style(obj, LV_BTN_STYLE_REL, &autonBtnStyle);

        obj = lv_label_create(obj, NULL);
        lv_label_set_text(obj, lowerAutons[i].name);
        lv_obj_set_style(obj, &autonBtnNameStyle);
    
    }

    lv_obj_set_size(selectedIndicator, 1.7 * tileLength, 1.2 * tileLength);
    lv_obj_set_style(selectedIndicator, &selectedIndicatorStyle);
    lv_obj_set_hidden(selectedIndicator, true);

    /*
     * Field
     */

    lv_obj_set_size(field, lv_obj_get_height(tabSpace), lv_obj_get_height(tabSpace));
    lv_obj_align(field, NULL, LV_ALIGN_IN_RIGHT_MID, 0, 0);
    lv_obj_set_style(field, &fieldStyle);

    // Field Tiles

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {

            // platform tiles
            if ((j == 0 || j == 5) && (i == 2 || i == 3)) {
                continue;
            }

            lv_obj_t* newTile = newButton(field, tileLength * j, tileLength * i, tileLength, tileLength,
                10 * (5 - i) + j, nullptr, false);
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

    tallNeutralMogo = newButton(field, 0, 0, 0.75 * tileLength, 0.75 * tileLength, 0, updateAutonTargets, false);
    lv_obj_set_free_ptr(tallNeutralMogo, this);
    lv_obj_align(tallNeutralMogo, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_btn_set_style(tallNeutralMogo, LV_BTN_STYLE_PR, &mogoStyle);
    lv_btn_set_style(tallNeutralMogo, LV_BTN_STYLE_REL, &mogoStyle);
    lv_obj_set_hidden(tallNeutralMogo, true);

    shortNeutralMogo = newButton(field, 0, 0, 0.75 * tileLength, 0.75 * tileLength, 1, updateAutonTargets, false);
    lv_obj_set_free_ptr(shortNeutralMogo, this);
    lv_obj_align(shortNeutralMogo, NULL, LV_ALIGN_CENTER, 0, 1.5 * tileLength);
    lv_btn_set_style(shortNeutralMogo, LV_BTN_STYLE_PR, &mogoStyle);
    lv_btn_set_style(shortNeutralMogo, LV_BTN_STYLE_REL, &mogoStyle);
    lv_obj_set_hidden(shortNeutralMogo, true);

    rings = newButton(field, 0, 0, 0.75 * tileLength, 0.5 * tileLength, 2, updateAutonTargets, false);
    lv_obj_set_free_ptr(rings, this);
    lv_obj_align(rings, NULL, LV_ALIGN_CENTER, -tileLength, 2 * tileLength);
    ringSelectedStyle.body.radius = lv_obj_get_height(rings) / 2;
    ringStyle.body.radius = lv_obj_get_height(rings) / 2;
    lv_btn_set_style(rings, LV_BTN_STYLE_PR, &ringStyle);
    lv_btn_set_style(rings, LV_BTN_STYLE_REL, &ringStyle);
    lv_obj_set_hidden(rings, true);

    /*
     * show default screen
     */

    if (defaultScreen) {
        changeTab(autonSelectionSwitch);
    } else {
        changeTab(odomSwitch);
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

}
#endif

void DisplayControl::updateOdomData(bool updateValues) {

    Drivetrain::Point odomData = Drivetrain::getPosition();

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