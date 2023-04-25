//
// Created by Joey Sorkin on 11/4/22.
//

#ifndef INC_7405MSPINUP_DISPLAY_H
#define INC_7405MSPINUP_DISPLAY_H

#ifndef DISPLAY
#define DISPLAY
#include "pros/apix.h"

class Display {
  static lv_theme_t* th;

 public:
  typedef enum auton_e {
    ALPHA = 0,
    BETA = 1,
    GAMMA = 2,
    DELTA = 3,
    OMEGA = 4,
  } auton_e_t;

 private:
  typedef struct ancestor_btn_s {
    lv_btn_ext_t btn;
    auton_e_t AutonType;
  } ancestor_btn_s_t;
  static std::unordered_map<auton_e_t, std::pair<std::string, lv_color_t>>
      AutonMap;

  static lv_obj_t* autonbtn;
  static lv_obj_t* autonbtn_label;

  static lv_res_t autonChange(lv_obj_t* button);
  static void create_home_tab(lv_obj_t* parent);

  static void renderGUI();

  static lv_obj_t* tabview;

 public:
  static void guiInitialize();
  static auton_e_t getAutonMode();
};

#endif
#endif  //INC_7405MSPINUP_DISPLAY_H