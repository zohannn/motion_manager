#include "../include/motion_manager/handposplot.hpp"


namespace motion_manager {


HandPosPlot::HandPosPlot(vector<vector<double>>& hand_pos)
{
    setTitle("Hand position [mm]");

    HandPosFunction hand_func(this,hand_pos);

    double x_min = hand_func.getX_min(); double x_max = hand_func.getX_max();
    double y_min = hand_func.getY_min(); double y_max = hand_func.getY_max();
    double z_min = hand_func.getZ_min()-5; double z_max = hand_func.getZ_max()+5;


    int mesh_size = hand_pos.size();
    hand_func.setMesh(mesh_size,mesh_size);
    hand_func.setDomain(x_min,x_max,y_min,y_max);
    hand_func.setMinZ(z_min);
    hand_func.setMaxZ(z_max);

    hand_func.create();

    setRotation(30,0,15);
    setScale(1,1,1);
    //setShift(0.15,0,0);
    //setZoom(0.8);
    setDataColor(new HandPosColor(this,z_min));

    for (unsigned i=0; i!=coordinates()->axes.size(); ++i)
    {
        coordinates()->axes[i].setAutoScale();
    }


    coordinates()->axes[X1].setLabelString("x");
    coordinates()->axes[X1].setNumberAnchor(BottomRight);
    coordinates()->axes[X1].adjustNumbers(10);
    coordinates()->axes[X1].setNumberFont(QString("times"),8);

    coordinates()->axes[Y1].setLabelString("y");
    coordinates()->axes[Y1].setNumberAnchor(BottomLeft);
    coordinates()->axes[Y1].adjustNumbers(20);
    coordinates()->axes[Y1].setNumberFont(QString("times"),8);

    coordinates()->axes[Z1].setLabelString("z");
    coordinates()->axes[Z1].setNumberAnchor(TopLeft);
    coordinates()->axes[Z1].adjustNumbers(20);
    coordinates()->axes[Z1].setNumberFont(QString("times"),8);

    setCoordinateStyle(BOX);
    //setFloorStyle(FLOORDATA);

    setPlotStyle(Dot(7.0,false));

    updateData();
    updateGL();

}

}// namespace motion_manager
