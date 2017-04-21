#include "setting.h"

int widowSize = 7;

const std::vector<Eigen::Vector2i>& trackModel(int mode) {
	static std::vector<Eigen::Vector2i> model0(8);
	model0[0](0) = 0; model0[0](1) = 1;
	model0[1](0) = 2; model0[1](1) = 0;
	model0[2](0) = 1; model0[2](1) = 1;
	model0[3](0) = 0; model0[3](1) = 2;
	model0[4](0) = -1; model0[4](1) = 1;
	model0[5](0) = -2; model0[5](1) = 0;
	model0[6](0) = -1; model0[6](1) = -1;
	model0[7](0) = 0; model0[7](1) = -2;

	return model0;
}