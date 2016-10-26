#include "../include/motion_manager/scenario.hpp"

namespace motion_manager{

Scenario::Scenario(string name, int id)
{
    this->m_name = name;
    this->m_scenarioID = id;
}


Scenario::Scenario(const Scenario &scene)
{

    this->m_name=scene.m_name;
    this->m_scenarioID=scene.m_scenarioID;

    if(!scene.objs_list.empty()){
        this->objs_list = std::vector<objectPtr>(scene.objs_list.size());
        std::copy(scene.objs_list.begin(),scene.objs_list.end(),this->objs_list.begin());
    }

    this->hPtr=humanoidPtr(new Humanoid(*scene.hPtr.get()));
}


Scenario::~Scenario()
{

}


void Scenario::setName(string &name)
{

    this->m_name = name;
}

void Scenario::setID(int id)
{

    this->m_scenarioID = id;
}

void Scenario::setObject(int pos, objectPtr obj)
{

    this->objs_list.at(pos) = objectPtr(new Object(*obj.get()));

}


string Scenario::getName()
{

    return this->m_name;
}

int Scenario::getID()
{

    return this->m_scenarioID;
}

humanoidPtr Scenario::getHumanoid()
{

    return hPtr;
}

bool Scenario::getObjects(vector<objectPtr> &objs)
{

    if(!this->objs_list.empty()){
        objs = std::vector<objectPtr>(this->objs_list.size());
        std::copy(this->objs_list.begin(),this->objs_list.end(),objs.begin());
        return true;
    }else{
        return false;
    }

}


void Scenario::addObject(Object* ob)
{

    this->objs_list.push_back(objectPtr(new Object(*ob->get())));
}

objectPtr Scenario::getObject(int pos)
{

    std::vector<objectPtr>::iterator ii = this->objs_list.begin();
    advance(ii,pos);

    return (*ii);

}

objectPtr Scenario::getObject(string obj_name)
{

    objectPtr obj = NULL;

    for(std::size_t i=0; i<this->objs_list.size();++i){
        string name = this->objs_list.at(i)->getName();
        if(boost::iequals(name,obj_name)){
            obj=this->objs_list.at(i);
            break;
        }
    }
    return obj;
}

void Scenario::addHumanoid(Humanoid* hh)
{

    this->hPtr = humanoidPtr(hh);
}



 } // namespace motion_manager
