#include "../include/motion_manager/target.hpp"

namespace motion_manager{


Target::Target()
{

}

Target::Target(string name, pos ppos, orient oor)
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

Target::Target(const Target &tar)
{
    this->m_name = tar.m_name;
    this->m_pos.Xpos = tar.m_pos.Xpos;
    this->m_pos.Ypos = tar.m_pos.Ypos;
    this->m_pos.Zpos = tar.m_pos.Zpos;
    this->m_or.pitch = tar.m_or.pitch;
    this->m_or.roll = tar.m_or.roll;
    this->m_or.yaw = tar.m_or.yaw;
    this->m_q = tar.m_q;
}

Target::~Target()
{


}

} // namespace motion_manager
