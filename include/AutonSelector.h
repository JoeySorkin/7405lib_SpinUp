#pragma once
#include <stdint.h>

extern "C" {
typedef struct _lv_obj_t lv_obj_t;
typedef int16_t lv_coord_t;
}

enum class Auton : uint32_t;

// half assed auton selector
class AutonSelector {
private:
	struct Button {
		lv_obj_t* btn;
		lv_obj_t* label;

		Button() : btn(nullptr), label(nullptr) {}
		Button(lv_obj_t* screen, const char* name, Auton auton);
		void setPos(lv_coord_t x, lv_coord_t y);
	};

	lv_obj_t* lcdScreen;
	lv_obj_t* selectorScreen;
	Button carry;
	Button left;
	Button right;

public:
	void initialize();

	// stalls the entire function until an auton is selected
	void showSelector();
	void restoreLCD();
};