#pragma once
#include <type_traits>
#include <vector>


class LUT {
public:
	LUT() = default;
	void add_data(double x_val, double y_val);
	enum interpolation_e { CONSTANT, LINEAR };
	double get_val(double x_val, interpolation_e interp_type = LINEAR);

private:
	double x_max{0};
	std::vector<std::pair<double, double>> values;
};
