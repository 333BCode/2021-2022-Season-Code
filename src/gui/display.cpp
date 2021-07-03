#include "gui/display.hpp"
#include "gui/button_callbacks.hpp"
#include "drivetrain.hpp"

#include <string>

DisplayControl displayControl {};

static lv_obj_t * newButton(lv_obj_t* parent, lv_coord_t xPos, lv_coord_t yPos, lv_coord_t width, lv_coord_t height,
    uint32_t freeNum, lv_res_t (*action)(lv_obj_t*), bool toggle)
{
    
    lv_obj_t* newBtn = lv_btn_create(parent, NULL);

    lv_obj_set_pos(newBtn, xPos, yPos);
    lv_obj_set_size(newBtn, width, height);
    lv_obj_set_free_num(newBtn, freeNum);
    lv_btn_set_action(newBtn, LV_BTN_ACTION_CLICK, action);
    lv_btn_set_toggle(newBtn, toggle);

    return newBtn;

}

static void setupStyle(lv_style_t* style, lv_style_t* copy, lv_color_t bodyColor,
    int16_t borderWidth, lv_color_t borderColor, lv_color_t textColor)
{

    lv_style_copy(style, copy);

    style->body.main_color = bodyColor;
    style->body.grad_color = bodyColor;

    style->body.border.width = borderWidth;
    style->body.radius = 0;
    style->body.border.color = borderColor;

    style->text.color = textColor;

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
    setupStyle(&fieldStyle, &lv_style_plain_color, LV_COLOR_GRAY, 1, LV_COLOR_WHITE, LV_COLOR_WHITE);
    
    setupStyle(&virtualBotStyle, &lv_style_plain, LV_COLOR_RED);
    virtualBotStyle.body.radius = LV_RADIUS_CIRCLE;
    lv_style_copy(&virtualBotDirectionIndicatorStyle, &lv_style_plain);
    virtualBotDirectionIndicatorStyle.line.width = 5;
    virtualBotDirectionIndicatorStyle.line.opa = LV_OPA_100;
    virtualBotDirectionIndicatorStyle.line.color = LV_COLOR_BLUE;

    lv_style_copy(&positionDataStyle, &lv_style_plain);
    positionDataStyle.text.color = LV_COLOR_WHITE;
    positionDataStyle.text.opa = LV_OPA_100;

    lv_style_copy(&pressedTextStyle, &positionDataStyle);
    pressedTextStyle.text.color = LV_COLOR_BLACK;

    setupStyle(&redPlatformStyle, &lv_style_plain_color, LV_COLOR_RED);
    setupStyle(&bluePlatformStyle, &lv_style_plain_color, LV_COLOR_BLUE);

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
    lv_btn_set_state(odomSwitch, LV_BTN_STATE_TGL_PR);

    odomSwitchText = lv_label_create(odomSwitch, NULL);
    lv_obj_align(odomSwitchText, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(odomSwitchText, "Odom");
    lv_obj_set_style(odomSwitchText, &pressedTextStyle);

    autonSelectionSwitch = newButton(tabSwitcher, 0, 0, lv_obj_get_width(tabSwitcher) / 3.1, tabBarHeight - 5, 
        1, changeTab, true);
    lv_obj_align(autonSelectionSwitch, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_free_ptr(autonSelectionSwitch, this);
    lv_btn_set_style(autonSelectionSwitch, LV_BTN_STYLE_TGL_PR, &tabviewBtnPressedStyle);
    lv_btn_set_style(autonSelectionSwitch, LV_BTN_STYLE_TGL_REL, &tabviewStyle);
    lv_btn_set_state(autonSelectionSwitch, LV_BTN_STATE_TGL_REL);

    autonSelectionSwitchText = lv_label_create(autonSelectionSwitch, NULL);
    lv_obj_align(autonSelectionSwitchText, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(autonSelectionSwitchText, "Auton");
    lv_obj_set_style(autonSelectionSwitchText, &positionDataStyle);

    debugSwitch = newButton(tabSwitcher, 0, 0, lv_obj_get_width(tabSwitcher) / 3.1, tabBarHeight - 5, 
        2, changeTab, true);
    lv_obj_align(debugSwitch, NULL, LV_ALIGN_IN_RIGHT_MID, -4, 0);
    lv_obj_set_free_ptr(debugSwitch, this);
    lv_btn_set_style(debugSwitch, LV_BTN_STYLE_TGL_PR, &tabviewBtnPressedStyle);
    lv_btn_set_style(debugSwitch, LV_BTN_STYLE_TGL_REL, &tabviewStyle);
    lv_btn_set_state(debugSwitch, LV_BTN_STATE_TGL_REL);

    debugSwitchText = lv_label_create(debugSwitch, NULL);
    lv_obj_align(debugSwitchText, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(debugSwitchText, "Debug");
    lv_obj_set_style(debugSwitchText, &positionDataStyle);

    lv_obj_set_size(tabSpace, lv_obj_get_width(lv_scr_act()), lv_obj_get_height(lv_scr_act()) - tabBarHeight);
    lv_obj_set_pos(tabSpace, 0, tabBarHeight);

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
    lv_obj_set_hidden(autonSelectionTab, true);

    /*
     * Debug Tab
     */
    
    lv_obj_set_size(debugTab, lv_obj_get_width(tabSpace), lv_obj_get_height(tabSpace));
    lv_obj_set_style(debugTab, &odomPageStyle);
    lv_obj_set_hidden(debugTab, true);

    /*
     * Field
     */

    lv_obj_set_size(field, lv_obj_get_height(tabSpace), lv_obj_get_height(tabSpace));
    lv_obj_align(field, NULL, LV_ALIGN_IN_RIGHT_MID, 0, 0);
    lv_obj_set_style(field, &fieldStyle);

    // Field Tiles
    tileLength = lv_obj_get_height(tabSpace) / 6.0;

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {

            // platform tiles
            if ((j == 0 || j == 5) && (i == 2 || i == 3)) {
                continue;
            }

            lv_obj_t* newTile = newButton(field, tileLength * j, tileLength * i, tileLength, tileLength,
                10 * i + j, setVirtualBotPos, false);
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
    virtualBotDirectionIndicator = lv_line_create(field, NULL);

    lv_led_on(virtualBot);
    lv_obj_set_size(virtualBot, tileLength / 1.5, tileLength / 1.5);
    lv_obj_set_style(virtualBot, &virtualBotStyle);

    lv_obj_set_style(virtualBotDirectionIndicator, &virtualBotDirectionIndicatorStyle);

}

DisplayControl::~DisplayControl() {
    lv_obj_clean(lv_scr_act());
}

void DisplayControl::updateOdomData(bool updateValues) {

    Drivetrain::XYHPoint odomData = Drivetrain::getPosition();
    // divide by 24 (144 / 6) instead of 144 since 6 * tile length = field length
    lv_obj_set_pos(virtualBot,
        (odomData.x / 24) * tileLength - 12, 186 - (odomData.y / 24) * tileLength);
    lv_obj_set_pos(virtualBotDirectionIndicator,
        (odomData.x / 24) * tileLength - 12, 186 - (odomData.y / 24) * tileLength);

    if (updateValues) {
        
        std::string newOdomReadout = "Values in inches\nand degrees:\n\n"
            "X Position: " + std::to_string(round(odomData.x        * 1000) / 1000).substr(0, 6) + "\n"
            "Y Position: " + std::to_string(round(odomData.y        * 1000) / 1000).substr(0, 6) + "\n"
            "Heading:    " + std::to_string(round(odomData.heading  * 1000) / 1000).substr(0, 6);

        lv_label_set_text(positionData, newOdomReadout.c_str());

    }

}