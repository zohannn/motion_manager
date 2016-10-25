#include "../include/motion_manager/target.hpp"

namespace motion_manager{


Target::Target()
{

}

Target::Target(string name, pos ppos, orient oor)
{
    this->m_name = name;
    this->m_pos = ppos;
    this->m_or = oor;
}

Target::Target(const Target &tar)
{
    this->m_name = tar.m_name;
    this->m_pos = tar.m_pos;
    this->m_or = tar.m_or;
}

Target::~Target()
{


}

} // namespace motion_manager
