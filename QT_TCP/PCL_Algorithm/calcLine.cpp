#include "calcLine.h"

//https://blog.csdn.net/qinqinxiansheng/article/details/104615531
//计算平面与平面的交线（附c++实现）
void calcLine(pcl::ModelCoefficients::Ptr coefsOfPlane1, pcl::ModelCoefficients::Ptr coefsOfPlane2, pcl::ModelCoefficients::Ptr coefsOfLine)
{
    //方向向量n=n1×n2=(b1*c2-c1*b2,c1*a2-a1*c2,a1*b2-b1*a2)
    double a1, b1, c1, d1, a2,b2, c2, d2;
    double tempy, tempz;
    a1= coefsOfPlane1->values[0];
    b1= coefsOfPlane1->values[1];
    c1= coefsOfPlane1->values[2];
    d1= coefsOfPlane1->values[3];
    a2= coefsOfPlane2->values[0];
    b2= coefsOfPlane2->values[1];
    c2= coefsOfPlane2->values[2];
    d2= coefsOfPlane2->values[3];
    tempz= -(d1 / b1 - d2 / b2) / (c1 / b1 - c2 / b2);
    tempy= (-c1 / b1)*tempz - d1 / b1;
    coefsOfLine->values.push_back(0.0);
    coefsOfLine->values.push_back(tempy);
    coefsOfLine->values.push_back(tempz);
    coefsOfLine->values.push_back(b1*c2 - c1*b2);
    coefsOfLine->values.push_back(c1*a2 - a1*c2);
    coefsOfLine->values.push_back(a1*b2 - b1*a2);
}
