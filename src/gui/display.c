#include "gui/display.h"
#include "display/lv_core/lv_obj.h"
#include "display/lv_misc/lv_area.h"
#include "display/lv_objx/lv_btn.h"
#include "display/lv_objx/lv_page.h"

/*
    static lv_obj_t* odomPage;
    static lv_obj_t* autonSelectionPage;
*/

static lv_obj_t * newButton(lv_coord_t xPos, lv_coord_t yPos, lv_coord_t width,
    lv_coord_t height, unsigned char (*action)(lv_obj_t*), const char* label = "")
{
    
    lv_obj_t* btn = lv_btn_create(lv_scr_act(), NULL);
    
    lv_obj_set_pos(btn, xPos, yPos);
    lv_obj_set_size(btn, width, height);
    lv_btn_set_action(btn, LV_BTN_ACTION_CLICK, action);

    lv_obj_t* text = lv_label_create(btn, NULL);
    
    lv_label_set_text(text, label);
    lv_obj_align(text, NULL, LV_ALIGN_CENTER, 0, 0);

    return btn;

}

void initDisplay() {



}