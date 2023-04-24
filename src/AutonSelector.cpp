#include "AutonSelector.h"
#include "Robot.h"
#include "display/lv_core/lv_obj.h"
#include "display/lv_objx/lv_btn.h"
#include "main.h"
#include <cstddef>

lv_style_t bRel;
lv_style_t bPr;
lv_style_t bTGL_Rel;
lv_style_t bTGL_Pr;

inline lv_coord_t percentX(double percent) {
	return 480 / 100 * percent;
}

inline lv_coord_t percentY(double percent) {
	return 240 / 100 * percent;
}

AutonSelector::Button::Button(lv_obj_t* screen, const char* name, Auton auton) {
	btn = lv_btn_create(screen, NULL);
	lv_btn_set_style(btn, LV_BTN_STYLE_REL, &bTGL_Pr);
	lv_obj_set_size(btn, 130, 200);
	lv_obj_set_free_num(btn, static_cast<uint32_t>(auton));
	lv_btn_set_action(btn, LV_BTN_ACTION_CLICK, [](lv_obj_t* btn) -> lv_res_t {
		uint32_t num = lv_obj_get_free_num(btn);
		Auton auton = static_cast<Auton>(num);
		sRobot->setAuton(auton);
		return LV_RES_OK;
	});

	label = lv_label_create(btn, NULL);
	lv_label_set_text(label, name);
}

void AutonSelector::Button::setPos(lv_coord_t x, lv_coord_t y) {
	lv_obj_set_pos(btn, x, y);
}

void AutonSelector::initialize() {
	lv_style_copy(&bRel, &lv_style_btn_rel);
	lv_style_copy(&bPr, &lv_style_btn_pr);
	lv_style_copy(&bTGL_Rel, &lv_style_btn_tgl_rel);
	lv_style_copy(&bTGL_Pr, &lv_style_btn_tgl_pr);

	bRel.body.opa = LV_OPA_TRANSP;
	bRel.body.radius = 0;
	bPr.body.opa = LV_OPA_TRANSP;
	bPr.body.radius = 0;
	bTGL_Rel.body.opa = LV_OPA_TRANSP;
	bTGL_Rel.body.radius = 0;
	bTGL_Pr.body.radius = 0;
	bTGL_Pr.body.opa = LV_OPA_TRANSP;

	pros::lcd::initialize();
	lcdScreen = lv_scr_act();

	// make auton selector screens
	selectorScreen = lv_obj_create(NULL, NULL);
	left = Button(selectorScreen, "Left", Auton::LEFT);
	left.setPos(percentX(5), percentY(10));
	right = Button(selectorScreen, "Right", Auton::RIGHT);
	right.setPos(percentX(75), percentY(10));
	carry = Button(selectorScreen, "Carry", Auton::CARRY);
	carry.setPos(percentX(35), percentY(10));
}

void AutonSelector::showSelector() {
	lv_scr_load(selectorScreen);

	while (sRobot->getAuton() == Auton::NONE) { pros::delay(10); }
}

void AutonSelector::restoreLCD() {
	lv_scr_load(lcdScreen);
}