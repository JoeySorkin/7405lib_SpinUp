//
// Created by Joey Sorkin on 11/4/22.
//

#include "Display.h"
#include "display/lv_core/lv_obj.h"
#include "display/lv_misc/lv_color.h"

lv_theme_t* Display::th;
lv_obj_t* Display::tabview;
lv_obj_t* Display::autonbtn;
lv_obj_t* Display::autonbtn_label;
std::unordered_map<Display::auton_e_t, std::pair<std::string, lv_color_t>>
    Display::AutonMap;

extern lv_font_t lv_font_dejavu_40;

void Display::guiInitialize() {
  th = lv_theme_alien_init(210, nullptr);
  AutonMap[auton_e_t::ALPHA] = std::pair<std::string, lv_color_t>(
      "No Auton", lv_color_hex(0xee5922));
  AutonMap[auton_e_t::BETA] = std::pair<std::string, lv_color_t>(
      "Left Auton", lv_color_hex(0x000000)); // 0xc18804
  AutonMap[auton_e_t::GAMMA] = std::pair<std::string, lv_color_t>(
      "Right Auton", lv_color_hex(0x008936)); // 0x008936 // 0xffffff
  AutonMap[auton_e_t::DELTA] = std::pair<std::string, lv_color_t>(
      "Carry Auton", lv_color_hex(0x334cb2)); // 0x1BC87B
   AutonMap[auton_e_t::OMEGA] =
       std::pair<std::string, lv_color_t>("Skills Auton", lv_color_hex(0x000000));

  renderGUI();
}

void Display::renderGUI() {
  lv_theme_set_current(th);

  /*Create a Tab view object*/
  tabview = lv_tabview_create(lv_scr_act(), nullptr);
  // Shorten Tab Height
  static lv_style_t shorterTabStyle;
  lv_style_copy(&shorterTabStyle, th->tabview.btn.bg);
  shorterTabStyle.body.padding.ver = -10;
  lv_tabview_set_style(tabview, LV_TABVIEW_STYLE_BTN_BG, &shorterTabStyle);

  // Shorten Indic Height
  static lv_style_t shorterTabIndicStyle;
  lv_style_copy(&shorterTabIndicStyle, th->tabview.indic);
  shorterTabIndicStyle.body.padding.inner = 5;
  lv_tabview_set_style(tabview, LV_TABVIEW_STYLE_INDIC, &shorterTabIndicStyle);

  // Create Tab Labels
  static lv_obj_t* tab1 = lv_tabview_add_tab(tabview, "AutonSwitcher");
  create_home_tab(tab1);
}
/*===============HOME TAB ===============*/

lv_res_t Display::autonChange(lv_obj_t* button) {
  auto* ext = static_cast<ancestor_btn_s_t*>(lv_obj_get_ext_attr(button));
  ext->AutonType = static_cast<auton_e_t>(
      ext->AutonType != AutonMap.size() - 1 ? ext->AutonType + 1 : 0);
  lv_style_t* btn_style = lv_obj_get_style(button);
  lv_label_set_text(lv_obj_get_child(button, nullptr),
                    AutonMap[ext->AutonType].first.c_str());
  lv_color_t btn_color = AutonMap[ext->AutonType].second;
  btn_style->body.main_color = btn_color;
  btn_style->body.grad_color = btn_color;
  return LV_RES_OK;
}

void Display::create_home_tab(lv_obj_t* parent) {
  lv_page_set_sb_mode(parent, LV_SB_MODE_OFF);

  //--button styles
  static lv_style_t good_lookin_buttons_REL;
  lv_style_copy(&good_lookin_buttons_REL, th->btn.rel);
  static lv_style_t good_lookin_buttons_PR;
  lv_style_copy(&good_lookin_buttons_PR, th->btn.pr);

  good_lookin_buttons_REL.body.radius = 0;
  good_lookin_buttons_REL.body.padding.ver = 5;
  good_lookin_buttons_REL.body.padding.hor = 15;

  good_lookin_buttons_PR.body.radius = 0;
  good_lookin_buttons_PR.body.padding.ver = 5;
  good_lookin_buttons_PR.body.padding.hor = 15;

  autonbtn = lv_btn_create(parent, nullptr);

  // magic
  lv_btn_set_action(autonbtn, LV_BTN_ACTION_CLICK,
                    autonChange); /*Assign an action function*/
  lv_obj_allocate_ext_attr(
      autonbtn, sizeof(ancestor_btn_s_t)); /*Re-alloacte the extended data*/
  auto* ext = static_cast<ancestor_btn_s_t*>(lv_obj_get_ext_attr(autonbtn));
  ext->AutonType = ALPHA;

  lv_obj_align(autonbtn, parent, LV_ALIGN_IN_BOTTOM_LEFT, 0, 20);

  // styles
  static lv_style_t squareButtonREL;
  lv_style_copy(&squareButtonREL, &lv_style_plain);
  squareButtonREL.body.padding.ver = 7;
  squareButtonREL.body.padding.hor = 0;
  squareButtonREL.body.radius = 2;

  static lv_style_t squareButtonPR;
  lv_style_copy(&squareButtonPR, th->btn.pr);
  squareButtonPR.body.padding.ver = 7;
  squareButtonPR.body.padding.hor = 0;
  squareButtonPR.body.radius = 2;

  lv_color_t btn_color = AutonMap[ext->AutonType].second;
  squareButtonREL.body.main_color = btn_color;
  squareButtonREL.body.grad_color = btn_color;

  lv_btn_set_style(autonbtn, LV_BTN_STYLE_REL, &squareButtonREL);
  lv_btn_set_style(autonbtn, LV_BTN_STYLE_PR, &squareButtonPR);
  lv_obj_set_size(autonbtn, 500, 150);
  autonbtn_label = lv_label_create(autonbtn, nullptr);
  static lv_style_t largeText;
  lv_style_copy(&largeText, th->label.sec);
  largeText.text.font = &lv_font_dejavu_40;  // make font big
  lv_obj_set_style(autonbtn_label, &largeText);
  lv_label_set_text(autonbtn_label, AutonMap[ext->AutonType].first.c_str());
}

Display::auton_e_t Display::getAutonMode() {
  auto* ext = static_cast<ancestor_btn_s_t*>(lv_obj_get_ext_attr(autonbtn));
  return ext->AutonType;
}