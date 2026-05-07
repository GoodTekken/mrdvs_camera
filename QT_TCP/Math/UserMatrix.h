#ifndef USERMATRIX_H
#define USERMATRIX_H

#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <QPoint>

#include "AGVMessage/CoordinateClass.h"

using namespace Eigen;

#define M_PI 3.14159265358979323846

Matrix<double, 4, 4> MatrixObject(int x, int y, int z, double angle);   //x,y,z坐标,角度
Matrix<double, 4, 4> MatrixObject(CoordinateClass temp_coordinate);

struct posture
{
public:
    int x;
    int y;
    int z;
    double angle;
};

posture GetPosture(Matrix<double, 4, 4> mymatrix);

QPoint GetMiddlePoint(QPoint startpoint,QPoint endpoint);

struct line_struct
{
    double RCA;
    double RCB;
    double angle;
};

line_struct TwoPointToLine(QPoint startPoint, QPoint endPoint);
double TwoPointToAngle(QPoint startPoint, QPoint endPoint);
double TwoPointToAngle(double startPoint_X, double startPoint_Y, double endPoint_X, double endPoint_Y);


#endif  //USERMATRIX_H
