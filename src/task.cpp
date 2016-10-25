#include "../include/motion_manager/task.hpp"

namespace motion_manager {


Task::Task()
{

}


Task::Task(const Task &t)
{

    this->prolem_list=t.prolem_list;
}


Task::~Task()
{

}

void Task::addProblem(Problem* s)
{

    this->prolem_list.push_back(problemPtr(s));

}

string Task::getProbInfo(int pos)
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

} // namespace motion_manager
