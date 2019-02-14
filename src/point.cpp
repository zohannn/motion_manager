#include "../include/motion_manager/point.hpp"

namespace motion_manager{


Point::Point()
{

}

Point::Point(string name, pos ppos, orient oor)
{
    this->m_name = name;
    this->m_pos.Xpos = ppos.Xpos;
    this->m_pos.Ypos = ppos.Ypos;
    this->m_pos.Zpos = ppos.Zpos;
    this->m_or.pitch = oor.pitch;
    this->m_or.roll = oor.roll;
    this->m_or.yaw = oor.yaw;
    this->m_q = AngleAxisd(oor.roll,Vector3d::UnitZ())*AngleAxisd(oor.pitch,Vector3d::UnitY())*AngleAxisd(oor.yaw,Vector3d::UnitX());
}

Point::Point(const Point &pt)
{
    this->m_name = pt.m_name;
    this->m_pos.Xpos = pt.m_pos.Xpos;
    this->m_pos.Ypos = pt.m_pos.Ypos;
    this->m_pos.Zpos = pt.m_pos.Zpos;
    this->m_or.pitch = pt.m_or.pitch;
    this->m_or.roll = pt.m_or.roll;
    this->m_or.yaw = pt.m_or.yaw;
    this->m_q = pt.m_q;
}

Point::~Point(){

}


void Point::setName(const string& name)
{

    this->m_name = name;
}


void Point::setPos(pos &ppos)
{

    this->m_pos = ppos;
}

void Point::setOr(orient &oor)
{

    this->m_or = oor;
    this->m_q = AngleAxisd(oor.roll, Vector3d::UnitZ())*AngleAxisd(oor.pitch,Vector3d::UnitY())*AngleAxisd(oor.yaw,Vector3d::UnitX());
}

void Point::setQuaternion(Quaterniond &or_q)
{
    this->m_q = or_q;
    Vector3d rpy = or_q.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX
    this->m_or.roll = rpy(0); this->m_or.pitch = rpy(1); this->m_or.yaw = rpy(2);
}


string Point::getName() const
{

    return this->m_name;
}

pos Point::getPos() const
{

    return this->m_pos;
}

orient Point::getOr() const
{

   return this->m_or;
}

Quaterniond Point::getQuaternion() const
{
    return this->m_q;
}


void Point::getXt(vector<double> &xt)
{

    this->getRotAxis(xt,0);
}


void Point::getYt(vector<double> &yt)
{

    this->getRotAxis(yt,1);
}

void Point::getZt(vector<double> &zt)
{

    this->getRotAxis(zt,2);
}

double Point::getNorm()
{

    return sqrt(pow((this->m_pos.Xpos),2)+pow((this->m_pos.Ypos),2)+pow((this->m_pos.Zpos),2));
}


void Point::RPY_matrix(Matrix3d &Rot)
{
    Rot = Matrix3d::Zero();

    double roll = this->m_or.roll; // around z
    double pitch = this->m_or.pitch; // around y
    double yaw = this->m_or.yaw; // around x

    // Rot = Rot_z * Rot_y * Rot_x

    Rot(0,0) = cos(roll)*cos(pitch);  Rot(0,1) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw); Rot(0,2) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
    Rot(1,0) = sin(roll)*cos(pitch);  Rot(1,1) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw); Rot(1,2) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
    Rot(2,0) = -sin(pitch);           Rot(2,1) = cos(pitch)*sin(yaw);                              Rot(2,2) = cos(pitch)*cos(yaw);

}

void Point::Trans_matrix(Matrix4d& Trans)
{

    Trans = Matrix4d::Zero();

    Matrix3d Rot;
    this->RPY_matrix(Rot);

    Trans(0,0) = Rot(0,0); Trans(0,1) = Rot(0,1); Trans(0,2) = Rot(0,2); Trans(0,3) = this->m_pos.Xpos;
    Trans(1,0) = Rot(1,0); Trans(1,1) = Rot(1,1); Trans(1,2) = Rot(1,2); Trans(1,3) = this->m_pos.Ypos;
    Trans(2,0) = Rot(2,0); Trans(2,1) = Rot(2,1); Trans(2,2) = Rot(2,2); Trans(2,3) = this->m_pos.Zpos;
    Trans(3,0) = 0;        Trans(3,1) = 0;        Trans(3,2) = 0;        Trans(3,3) = 1;

}


void Point::getRotAxis(vector<double>& xt, int id){

    Matrix3d Rot;
    this->RPY_matrix(Rot);
    Vector3d v = Rot.col(id);

    // get the components of the axis
    xt.push_back(v(0)); // x
    xt.push_back(v(1)); // y
    xt.push_back(v(2)); // z
}

}// namespace motion_manager
