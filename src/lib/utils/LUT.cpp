#include "lib/utils/LUT.h"
#include "lib/utils/Math.h"
#include <algorithm>
#include <cmath>

void LUT::add_data(double x_val, double y_val) {
	x_max = fmax(x_val, x_max);
	values.emplace_back(x_val, y_val);
}
double LUT::get_val(double x_val, interpolation_e interp_type) {
	std::sort(values.begin(), values.end());
	switch (interp_type) {
		case CONSTANT:
			if (x_val <= x_max && x_val >= 0.0) {
				for (std::size_t i = 0; i < values.size() - 1; ++i) {
					if (x_val >= values[i].first && values[i + 1].first >= x_val) {
						if (util::fpEquality(x_val, 0.0)) return 0;
						return values[i].second;
					}
				}
			}
			if (x_val > x_max) {
				return values.back().second;
			} else {
				return 0;
			}
		case LINEAR:
			if (x_val <= x_max && x_val >= 0.0) {
				for (std::size_t i = 0; i < values.size() - 1; ++i) {
					if (x_val >= values[i].first && values[i + 1].first >= x_val) {
						if (util::fpEquality(x_val, 0.0)) return 0;
						double t_val = (x_val - values[i].first) / (values[i + 1].first - values[i].first);
						return util::lerp(values[i].second, values[i + 1].second, t_val);
					}
				}
			}
			if (x_val > x_max) {
				return values.back().second;
			} else {
				return std::numeric_limits<double>::signaling_NaN();
			}
		default:
			return 0;
	}
}
