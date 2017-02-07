#include "../include/motion_manager/handposcolor.hpp"

namespace motion_manager{


HandPosColor::HandPosColor(SurfacePlot* pw,double z_min)
    :StandardColor(pw)
{
    z_min_=z_min;
}

RGBA HandPosColor::operator ()(double x, double y, double z) const
{
    if(z<=z_min_){
        return RGBA(1,1,1); // white
    }else{
        return RGBA(0,0,1); // blue
    }

}

}// namespace motion_manager
