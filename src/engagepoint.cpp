#include "../include/motion_manager/engagepoint.hpp"

namespace motion_manager{


EngagePoint::EngagePoint()
{

}

EngagePoint::EngagePoint(string name, pos ppos, orient oor)
{
    this->m_name = name;
    this->m_pos.Xpos = ppos.Xpos;
    this->m_pos.Ypos = ppos.Ypos;
    this->m_pos.Zpos = ppos.Zpos;
    this->m_or.pitch = oor.pitch;
    this->m_or.roll = oor.roll;
    this->m_or.yaw = oor.yaw;
}

EngagePoint::EngagePoint(const EngagePoint &eng)
{
    this->m_name = eng.m_name;
    this->m_pos.Xpos = eng.m_pos.Xpos;
    this->m_pos.Ypos = eng.m_pos.Ypos;
    this->m_pos.Zpos = eng.m_pos.Zpos;
    this->m_or.pitch = eng.m_or.pitch;
    this->m_or.roll = eng.m_or.roll;
    this->m_or.yaw = eng.m_or.yaw;
}

EngagePoint::~EngagePoint()
{


}

} // namespace motion_manager
