#include "../include/motion_manager/handposfunction.hpp"

namespace motion_manager {

HandPosFunction::HandPosFunction(SurfacePlot* pw, vector<vector<double>>& hand_pos)
    : Function(pw)
{
    handPos = hand_pos;
    //plotted.resize(hand_pos.size(),false);
}

double HandPosFunction::getX_min()
{
    vector<double> handPos_x;
    for(size_t i=0;i<handPos.size();++i)
    {
        vector<double> hand_point = handPos.at(i);
        handPos_x.push_back(hand_point.at(0));
    }
    double min = *min_element(handPos_x.begin(), handPos_x.end());
    return min;
}

double HandPosFunction::getX_max()
{

    vector<double> handPos_x;
    for(size_t i=0;i<handPos.size();++i)
    {
        vector<double> hand_point = handPos.at(i);
        handPos_x.push_back(hand_point.at(0));
    }
    double max = *max_element(handPos_x.begin(), handPos_x.end());
    return max;
}

double HandPosFunction::getY_min()
{
    vector<double> handPos_y;
    for(size_t i=0;i<handPos.size();++i)
    {
        vector<double> hand_point = handPos.at(i);
        handPos_y.push_back(hand_point.at(1));
    }
    double min = *min_element(handPos_y.begin(), handPos_y.end());
    return min;
}

double HandPosFunction::getY_max()
{
    vector<double> handPos_y;
    for(size_t i=0;i<handPos.size();++i)
    {
        vector<double> hand_point = handPos.at(i);
        handPos_y.push_back(hand_point.at(1));
    }
    double max = *max_element(handPos_y.begin(), handPos_y.end());
    return max;
}

double HandPosFunction::getZ_min()
{
    vector<double> handPos_z;
    for(size_t i=0;i<handPos.size();++i)
    {
        vector<double> hand_point = handPos.at(i);
        handPos_z.push_back(hand_point.at(2));
    }
    double min = *min_element(handPos_z.begin(), handPos_z.end());
    return min;
}

double HandPosFunction::getZ_max()
{
    vector<double> handPos_z;
    for(size_t i=0;i<handPos.size();++i)
    {
        vector<double> hand_point = handPos.at(i);
        handPos_z.push_back(hand_point.at(2));
    }
    double max = *max_element(handPos_z.begin(), handPos_z.end());
    return max;
}

double HandPosFunction::operator ()(double x, double y)
{
   double z = this->getZ_min()-5;
   /**
   vector<double> handPos_x; vector<double> handPos_y; vector<double> handPos_z;
   for(size_t i=0;i<handPos.size();++i)
   {
       vector<double> hand_point = handPos.at(i);
       handPos_x.push_back(hand_point.at(0));
       handPos_y.push_back(hand_point.at(1));
       handPos_z.push_back(hand_point.at(2));
   }
   **/
   for(size_t i=0; i < handPos.size();++i){
       vector<double> hand_point = handPos.at(i);
       double x_curr = hand_point.at(0);
       double y_curr = hand_point.at(1);
       double z_curr = hand_point.at(2);
       //if((pow((x-x_curr),2)<=0.001) && (pow((y-y_curr),2)<=0.001)){
       if((x==x_curr) && (y==y_curr)){// && (plotted.at(i)==false)){
            z=z_curr;
            //plotted.at(i)==true;
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
    std::sort (handPos_x.begin(), handPos_x.end()); // sort into ascending order
    std::sort (handPos_y.begin(), handPos_y.end()); // sort into ascending order
    x_min = getX_min(); x_max = getX_max();
    y_min = getY_min(); y_max = getY_max();
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
        ((SurfacePlot*)plotwidget_p)->loadFromData(data, handPos_x.size(), handPos_y.size(), x_min, x_max, y_min, y_max);
    }

    for ( size_t i = 0; i < handPos_x.size(); i++)
    {
        delete [] data[i];
    }

    delete [] data;



    return true;
}

}// namespace motion_manager
