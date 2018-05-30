#include "../include/motion_manager/task.hpp"

namespace motion_manager {


Task::Task()
{
    this->arm_code = 1;
}


Task::Task(const Task &t)
{

    this->prolem_list=t.prolem_list;
    this->arm_code = t.arm_code;
}


Task::~Task()
{

}

void Task::addProblem(Problem* s)
{

    this->prolem_list.push_back(problemPtr(new Problem(*s)));

}

string Task::getProblemInfo(int pos)
{

    vector<problemPtr>::iterator ii = this->prolem_list.begin();
    advance(ii,pos);
    return (*ii)->getInfoLine();
}

int Task::getProblemNumber()
{

    return this->prolem_list.size();
}

problemPtr Task::getProblem(int pos)
{

    vector<problemPtr>::iterator ii = this->prolem_list.begin();
    advance(ii,pos);

    return (*ii);
}


void Task::clearProblems()
{

    if(!this->prolem_list.empty()){
        this->prolem_list.clear();
    }
}

int Task::getArm()
{
    return this->arm_code;
}

void Task::setArm(int a)
{
    this->arm_code = a;
}

} // namespace motion_manager
