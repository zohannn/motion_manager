#ifndef HANDPOSFUNCTION_H
#define HANDPOSFUNCTION_H

#include <qwt3d_surfaceplot.h>
#include <qwt3d_function.h>
#include <qwt3d_plot.h>
#include <qwt3d_enrichment_std.h>

using namespace Qwt3D;
using namespace std;

namespace motion_manager {

class HandPosFunction: public Function
{
public:
    HandPosFunction(SurfacePlot* pw, vector<vector<double>>& hand_pos);

    double operator()(double x, double y);
    bool create();

    double getX_min();/**< get the minimum of the x coordinate of the hand position */
    double getX_max();/**< get the maximum of the x coordinate of the hand position */
    double getY_min();/**< get the minimum of the y coordinate of the hand position */
    double getY_max();/**< get the maximum of the y coordinate of the hand position */
    double getZ_min();/**< get the minimum of the z coordinate of the hand position */
    double getZ_max();/**< get the maximum of the z coordinate of the hand position */


private:
    vector<vector<double>> handPos; /**< hand position. 0=x,1=y,2=z*/
    double x_min;/**< minimum value of the x coordinate of the hand position */
    double x_max;/**< maximum value of the x coordinate of the hand position */
    double y_min;/**< minimum value of the y coordinate of the hand position */
    double y_max;/**< maximum value of the y coordinate of the hand position */
    double z_min;/**< minimum value of the z coordinate of the hand position */
    double z_max;/**< maximum value of the z coordinate of the hand position */
    //vector<bool> plotted;
};

}// namespace motion_manager
#endif // HANDPOSFUNCTION_H
