#include "../include/motion_manager/handposfunction.hpp"

namespace motion_manager {

HandPosFunction::HandPosFunction(SurfacePlot* pw, vector<vector<double>>& hand_pos)
    : Function(pw)
{
    handPos = hand_pos;
    plotted.resize(hand_pos.size(),false);
}

double HandPosFunction::getX_min()
{
    double min = 1e+50;
    for(size_t i=0; i < handPos.size();++i){
        vector<double> hand_point = handPos.at(i);
        double curr = hand_point.at(0);
        if(min > curr){
            min = curr;
        }
    }
    return min;
}

double HandPosFunction::getX_max()
{
    double max = 1e-50;
    for(size_t i=0; i < handPos.size();++i){
        vector<double> hand_point = handPos.at(i);
        double curr = hand_point.at(0);
        if(max < curr){
            max = curr;
        }
    }
    return max;
}

double HandPosFunction::getY_min()
{
    double min = 1e+50;
    for(size_t i=0; i < handPos.size();++i){
        vector<double> hand_point = handPos.at(i);
        double curr = hand_point.at(1);
        if(min > curr){
            min = curr;
        }
    }
    return min;
}

double HandPosFunction::getY_max()
{
    double max = 1e-50;
    for(size_t i=0; i < handPos.size();++i){
        vector<double> hand_point = handPos.at(i);
        double curr = hand_point.at(1);
        if(max < curr){
            max = curr;
        }
    }
    return max;
}

double HandPosFunction::getZ_min()
{
    double min = 1e+50;
    for(size_t i=0; i < handPos.size();++i){
        vector<double> hand_point = handPos.at(i);
        double curr = hand_point.at(2);
        if(min > curr){
            min = curr;
        }
    }
    return min;
}

double HandPosFunction::getZ_max()
{
    double max = 1e-50;
    for(size_t i=0; i < handPos.size();++i){
        vector<double> hand_point = handPos.at(i);
        double curr = hand_point.at(2);
        if(max < curr){
            max = curr;
        }
    }
    return max;
}

double HandPosFunction::operator ()(double x, double y)
{
   double z = this->getZ_min()-5;
   vector<double> handPos_x; vector<double> handPos_y; vector<double> handPos_z;
   for(size_t i=0;i<handPos.size();++i)
   {
       vector<double> hand_point = handPos.at(i);
       handPos_x.push_back(hand_point.at(0));
       handPos_y.push_back(hand_point.at(1));
       handPos_z.push_back(hand_point.at(2));
   }

   for(size_t i=0; i < handPos.size();++i){
       vector<double> hand_point = handPos.at(i);
       double x_curr = hand_point.at(0);
       double y_curr = hand_point.at(1);
       double z_curr = hand_point.at(2);
       //if((pow((x-x_curr),2)<=0.001) && (pow((y-y_curr),2)<=0.001)){
       if((x==x_curr) && (y==y_curr) && (plotted.at(i)==false)){
            z=z_curr;
            plotted.at(i)==true;
            break;
       }
   }
   return z;
}

bool HandPosFunction::create()
{
    vector<double> handPos_x; vector<double> handPos_y;
    for(size_t i=0;i<handPos.size();++i)
    {
        vector<double> hand_point = handPos.at(i);
        handPos_x.push_back(hand_point.at(0));
        handPos_y.push_back(hand_point.at(1));
    }

    if ((handPos_x.size()<=2) || (handPos_y.size()<=2) || !plotwidget_p)
        return false;

    /* allocate some space for the mesh */
    double** data         = new double* [handPos_x.size()] ;

    for ( size_t i = 0; i < handPos_x.size(); i++)
    {
        data[i]         = new double [handPos_y.size()];
    }

    /* get the data */
    for (size_t i = 0; i < handPos_x.size(); ++i)
    {
        for (size_t j = 0; j < handPos_y.size(); ++j)
        {
            data[i][j] = operator()(handPos_x.at(i), handPos_y.at(j));

            if (data[i][j] > range_p.maxVertex.z)
                data[i][j] = range_p.maxVertex.z;
            else if (data[i][j] < range_p.minVertex.z)
                data[i][j] = range_p.minVertex.z;
        }
    }

    Q_ASSERT(plotwidget_p);
    if (!plotwidget_p)
    {
        fprintf(stderr,"Function: no valid Plot3D Widget assigned");
    }
    else
    {
        ((SurfacePlot*)plotwidget_p)->loadFromData(data, umesh_p, vmesh_p, minu_p, maxu_p, minv_p, maxv_p);
    }

    for ( size_t i = 0; i < handPos_x.size(); i++)
    {
        delete [] data[i];
    }

    delete [] data;



    return true;
}

}// namespace motion_manager
