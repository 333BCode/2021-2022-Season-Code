#include "gui/display.hpp"
#include "display/lv_core/lv_obj.h"
#include "display/lv_core/lv_style.h"
#include "display/lv_objx/lv_tabview.h"
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
    
    : tabview {lv_tabview_create(lv_scr_act(), NULL)},
    odomTab {lv_tabview_add_tab(tabview, "Field")}, autonSelectionTab {lv_tabview_add_tab(tabview, "Auton")},
    
    odomPage {lv_obj_create(odomTab, NULL)}, autonSelectionPage {lv_obj_create(autonSelectionTab, NULL)},
    
    field {lv_obj_create(odomPage, NULL)},
    virtualBot {lv_led_create(field, NULL)}, virtualBotDirectionIndicator {lv_line_create(field, NULL)},
    positionData {lv_label_create(odomPage, NULL)}

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

    /**
     * Set up tabview
     */

    lv_tabview_set_style(tabview, LV_TABVIEW_STYLE_INDIC, &tabviewStyleIndic);
    lv_tabview_set_style(tabview, LV_TABVIEW_STYLE_BTN_REL, &tabviewStyle);
    lv_tabview_set_style(tabview, LV_TABVIEW_STYLE_BTN_PR, &tabviewBtnPressedStyle);

    lv_obj_set_style(odomTab, &tabviewStyle);
    lv_obj_set_style(autonSelectionTab, &tabviewStyle);

    /**
     * Set up odomTab
     */
    
    lv_obj_set_size(odomPage, lv_obj_get_width(odomTab), lv_obj_get_height(odomTab));
    lv_obj_align(odomPage, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style(odomPage, &odomPageStyle);

    // Field
    lv_obj_set_size(field, lv_obj_get_height(odomPage) - 5, lv_obj_get_height(odomPage) - 5);
    lv_obj_align(field, NULL, LV_ALIGN_IN_RIGHT_MID, 0, 0);
    lv_obj_set_style(field, &fieldStyle);

    // Field Tiles
    tileLength = lv_obj_get_height(odomPage) / 6.0;

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            lv_obj_t* newTile = newButton(field, tileLength * j, tileLength * i, tileLength, tileLength,
                10 * i + j, setVirtualBotPos, false);
            lv_btn_set_style(newTile, LV_BTN_STYLE_PR, &fieldStyle);
            lv_btn_set_style(newTile, LV_BTN_STYLE_REL, &fieldStyle);
        }
    }

    // Virtual Bot
    lv_led_on(virtualBot);
    lv_obj_set_size(virtualBot, tileLength / 1.5, tileLength / 1.5);
    lv_obj_set_style(virtualBot, &virtualBotStyle);

    lv_obj_set_style(virtualBotDirectionIndicator, &virtualBotDirectionIndicatorStyle);

    // Position Data
    lv_obj_align(positionData, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);
    lv_label_set_text(positionData, "Odom starting up...");
    lv_obj_set_style(positionData, &positionDataStyle);

    /**
     * Set up autonSelectionTab
     */

}

DisplayControl::~DisplayControl() {
    lv_obj_clean(lv_scr_act());
}

void DisplayControl::updateOdomData() {

    std::array<long double, 3> odomData = Drivetrain::getPosition();
    // divide by 24 (144 / 6) instead of 144 since 6 * tile length = field length
    lv_obj_set_pos(virtualBot,
        (odomData[0] / 24) * tileLength, (odomData[1] / 24) * tileLength);
    lv_obj_set_pos(virtualBotDirectionIndicator,
        (odomData[0] / 24) * tileLength, (odomData[1] / 24) * tileLength);

    std::string newOdomReadout = "Values in inches and degrees:\n"
        "X Position: " + std::to_string(round(odomData[0] * 1000) / 1000).substr(0, 6) + "\n"
        "Y Position: " + std::to_string(round(odomData[1] * 1000) / 1000).substr(0, 6) + "\n"
        "Heading:    " + std::to_string(round(odomData[2] * 1000) / 1000).substr(0, 6);

    lv_label_set_text(positionData, newOdomReadout.c_str());

}