#include "../include/motion_manager/pose.hpp"

namespace motion_manager{


Pose::Pose()
{

}

Pose::Pose(string name, pos ppos, orient oor)
{
    this->m_name = name;
    this->m_pos.Xpos = ppos.Xpos;
    this->m_pos.Ypos = ppos.Ypos;
    this->m_pos.Zpos = ppos.Zpos;
    this->m_or.pitch = oor.pitch;
    this->m_or.roll = oor.roll;
    this->m_or.yaw = oor.yaw;
    this->obj_related = false;
    this->obj_id = -1;
}

Pose::Pose(string name, pos ppos, orient oor, bool obj_rel, int id)
{
    this->m_name = name;
    this->m_pos.Xpos = ppos.Xpos;
    this->m_pos.Ypos = ppos.Ypos;
    this->m_pos.Zpos = ppos.Zpos;
    this->m_or.pitch = oor.pitch;
    this->m_or.roll = oor.roll;
    this->m_or.yaw = oor.yaw;
    this->obj_related = obj_rel;
    this->obj_id = id;
}


Pose::Pose(const Pose &pose)
{
    this->m_name = pose.m_name;
    this->m_pos.Xpos = pose.m_pos.Xpos;
    this->m_pos.Ypos = pose.m_pos.Ypos;
    this->m_pos.Zpos = pose.m_pos.Zpos;
    this->m_or.pitch = pose.m_or.pitch;
    this->m_or.roll = pose.m_or.roll;
    this->m_or.yaw = pose.m_or.yaw;
    this->obj_related = pose.obj_related;
    this->obj_id = pose.obj_id;
}

Pose::~Pose()
{


}


void Pose::setObjRel(bool obj_rel)
{
    this->obj_related = obj_rel;
}

void Pose::setObjId(int id)
{
    this->obj_id = id;
}

bool Pose::getObjRel()
{
    return this->obj_related;
}

int Pose::getObjId()
{
    return this->obj_id;
}

} // namespace motion_manager
