#include "../include/motion_manager/scenario.hpp"

namespace motion_manager{

Scenario::Scenario(string name, int id)
{
    this->m_name = name;
    this->m_scenarioID = id;
    this->hand_target = targetPtr(new Target());
    this->hand_pose = posePtr(new Pose());
}


Scenario::Scenario(const Scenario &scene)
{

    this->m_name=scene.m_name;
    this->m_scenarioID=scene.m_scenarioID;

    this->objs_list.clear();
    if(!scene.objs_list.empty()){
        for(size_t i=0; i<scene.objs_list.size();++i){
           objectPtr obj = scene.objs_list.at(i);
           this->objs_list.push_back(objectPtr(new Object(*obj.get())));
        }
    }
    this->poses_list.clear();
    if(!scene.poses_list.empty()){
        for(size_t i=0; i<scene.poses_list.size();++i){
           posePtr pt = scene.poses_list.at(i);
           this->poses_list.push_back(posePtr(new Pose(*pt.get())));
        }
    }

    this->hPtr=humanoidPtr(new Humanoid(*scene.hPtr.get()));

    this->hand_target = targetPtr(new Target(*scene.hand_target.get()));
    this->hand_pose = posePtr(new Pose(*scene.hand_pose.get()));
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

void Scenario::setPose(int pos, posePtr pt)
{

    this->poses_list.at(pos) = posePtr(new Pose(*pt.get()));

}

void Scenario::setHandTarget(targetPtr h_tar)
{

    this->hand_target = targetPtr(new Target(*h_tar.get()));
}

void Scenario::setHandPose(posePtr h_pose)
{

    this->hand_pose = posePtr(new Pose(*h_pose.get()));
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

bool Scenario::getPoses(vector<posePtr> &pts)
{

    if(!this->poses_list.empty()){
        pts = std::vector<posePtr>(this->poses_list.size());
        std::copy(this->poses_list.begin(),this->poses_list.end(),pts.begin());
        return true;
    }else{
        return false;
    }

}

targetPtr Scenario::getHandTarget()
{

    return this->hand_target;
}

posePtr Scenario::getHandPose()
{

    return this->hand_pose;
}



void Scenario::addObject(objectPtr obj_ptr)
{

    this->objs_list.push_back(objectPtr(new Object(*obj_ptr.get())));
}

void Scenario::addPose(posePtr pose_ptr)
{
    this->poses_list.push_back(posePtr(new Pose(*pose_ptr.get())));
}

objectPtr Scenario::getObject(int pos)
{

    std::vector<objectPtr>::iterator ii = this->objs_list.begin();
    advance(ii,pos);

    return (*ii);

}

posePtr Scenario::getPose(int pos)
{
    std::vector<posePtr>::iterator ii = this->poses_list.begin();
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

posePtr Scenario::getPose(string pose_name)
{

    posePtr pose = NULL;

    for(std::size_t i=0; i<this->poses_list.size();++i){
        string name = this->poses_list.at(i)->getName();
        if(boost::iequals(name,pose_name)){
            pose=this->poses_list.at(i);
            break;
        }
    }
    return pose;
}


void Scenario::addHumanoid(humanoidPtr hh_ptr)
{

    this->hPtr = humanoidPtr(new Humanoid(*hh_ptr.get()));
}



 } // namespace motion_manager
