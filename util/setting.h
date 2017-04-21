#ifndef SETTING_H
#define SETTING_H

#include <Eigen/Dense>
#include <vector>

#define IMG_LEVEL                         5
#define IMUMEASURE_BETWEEN_FRAME_MAX      100

#define fast_threshold   5.0
#define edge_threshold   20.0


#define ThreadNum         4
#define detectCellWidth   4
#define detectCellHeight  4
#define detectWidthGrid   4
#define detectHeightGrid  4
#define Gravity           9.78
#define IuminanceErr      30

const std::vector<Eigen::Vector2i>& trackModel(int mode = 0);
extern int widowSize;


#define SIMPLE_BA         0x000000001

#endif // SETTING_H
