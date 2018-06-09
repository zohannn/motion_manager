#include "../include/motion_manager/movement.hpp"



namespace motion_manager {


Movement::Movement(int type, int arm)
{

    this->obj=objectPtr(new Object());
    this->obj_init=objectPtr(new Object());
    this->obj_eng=objectPtr(new Object());
    this->pose = posePtr(new Pose());

    this->obj_left=objectPtr(new Object());
    this->obj_init_left=objectPtr(new Object());
    this->obj_eng_left=objectPtr(new Object());
    this->pose_left = posePtr(new Pose());
    this->grip_str_left=string("No Grip");
    this->prec_left=false;


    this->type=type;
    switch (type){

    case 0:

        this->strType = string("Reach-to-grasp");
        break;

    case 1:

        this->strType = string("Reaching");
        break;

    case 2:

        this->strType = string("Transport");
        break;

    case 3:

        this->strType = string("Engage");
        break;

    case 4:

        this->strType = string("Disengage");
        break;

    case 5:

        this->strType = string("Go park");
        break;


    }


    //this->grip_code = 0;
    this->prec=false;
    this->arm = arm;
    this->executed = false;

}


Movement::Movement(int type, int arm, objectPtr obj)
{

    this->type=type;
    switch (type){

    case 0:

        this->strType = string("Reach-to-grasp");
        break;

    case 1:

        this->strType = string("Reaching");
        break;

    case 2:

        this->strType = string("Transport");
        break;

    case 3:

        this->strType = string("Engage");
        break;

    case 4:

        this->strType = string("Disengage");
        break;

    case 5:

        this->strType = string("Go park");
        break;


    }

    this->obj = obj;
    this->obj_init = obj;
    this->obj_eng=objectPtr(new Object());
    this->pose=posePtr(new Pose());
    //this->grip_code = 0;
    this->prec=false;
    this->grip_str=string("No Grip");
    this->arm=arm;
    this->executed = false;

    this->obj_left=objectPtr(new Object());
    this->obj_init_left=objectPtr(new Object());
    this->obj_eng_left=objectPtr(new Object());
    this->pose_left = posePtr(new Pose());
    this->grip_str_left=string("No Grip");
    this->prec_left=false;


}

Movement::Movement(int type, int arm, posePtr pose)
{

    this->type=type;
    switch (type){

    case 0:

        this->strType = string("Reach-to-grasp");
        break;

    case 1:

        this->strType = string("Reaching");
        break;

    case 2:

        this->strType = string("Transport");
        break;

    case 3:

        this->strType = string("Engage");
        break;

    case 4:

        this->strType = string("Disengage");
        break;

    case 5:

        this->strType = string("Go park");
        break;


    }

    this->obj = objectPtr(new Object());
    this->obj_init = objectPtr(new Object());
    this->obj_eng=objectPtr(new Object());
    this->pose=pose;
    //this->grip_code = 0;
    this->prec=false;
    this->grip_str=string("No Grip");
    this->arm=arm;
    this->executed = false;

    this->obj_left=objectPtr(new Object());
    this->obj_init_left=objectPtr(new Object());
    this->obj_eng_left=objectPtr(new Object());
    this->pose_left = posePtr(new Pose());
    this->grip_str_left=string("No Grip");
    this->prec_left=false;



}


Movement::Movement(int type, int arm, objectPtr obj, bool prec)
{

    this->type=type;
    switch (type){
    case 0:
        this->strType = string("Reach-to-grasp");
        break;
    case 1:
        this->strType = string("Reaching");
        break;
    case 2:
        this->strType = string("Transport");
        break;
    case 3:
        this->strType = string("Engage");
        break;
    case 4:
        this->strType = string("Disengage");
        break;
    case 5:
        this->strType = string("Go park");
        break;
    }


    this->obj = obj;
    this->obj_init = obj;
    this->obj_eng=objectPtr(new Object());
    this->pose = posePtr(new Pose());

    this->prec=prec;
    if(prec){
        this->grip_str=string("Precision");
    }else{
        this->grip_str=string("Full");
    }
    /**
    if (prec){
        // precision grip
        switch (grip_id) {
        case 0:
            this->grip_code = 111; // Side thumb left
            this->grip_str=string("Precision Side thumb left");
            break;
        case 1:
            this->grip_code = 112; // Side thumb right
            this->grip_str=string("Precision Side thumb right");
            break;
        case 2:
            this->grip_code = 113; // Side thumb up
            this->grip_str=string("Precision Side thumb up");
            break;
        case 3:
            this->grip_code = 114; // Side thumb down
            this->grip_str=string("Precision Side thumb down");
            break;
        case 4:
            this->grip_code = 121; // Above
            this->grip_str=string("Precision Above");
            break;
        case 5:
            this->grip_code = 122; // Below
            this->grip_str=string("Precision Below");
            break;
        case 6:
            this->grip_code = 0;
            this->grip_str=string("No Grip");
            break;

        }
    }else{
        //full grip

        switch (grip_id) {
        case 0:
            this->grip_code = 211; // Side thumb left
            this->grip_str=string("Full Side thumb left");
            break;
        case 1:
            this->grip_code = 212; // Side thumb right
            this->grip_str=string("Full Side thumb right");
            break;
        case 2:
            this->grip_code = 213; // Side thumb up
            this->grip_str=string("Full Side thumb up");
            break;
        case 3:
            this->grip_code = 214; // Side thumb down
            this->grip_str=string("Full Side thumb down");
            break;
        case 4:
            this->grip_code = 221; // Above
            this->grip_str=string("Full Above");
            break;
        case 5:
            this->grip_code = 222; // Below
            this->grip_str=string("Full Below");
            break;
        case 6:
            this->grip_code = 0;
            this->grip_str=string("No Grip");
            break;

        }
    }
    */

    this->arm=arm;
    this->executed = false;

    this->obj_left=objectPtr(new Object());
    this->obj_init_left=objectPtr(new Object());
    this->obj_eng_left=objectPtr(new Object());
    this->pose_left = posePtr(new Pose());
    this->grip_str_left=string("No Grip");
    this->prec_left=false;

}


Movement::Movement(int type, int arm,objectPtr obj, objectPtr obj_eng,bool prec)
{

    this->type=type;
    switch (type){
    case 0:
        this->strType = string("Reach-to-grasp");
        break;
    case 1:
        this->strType = string("Reaching");
        break;
    case 2:
        this->strType = string("Transport");
        break;
    case 3:
        this->strType = string("Engage");
        break;
    case 4:
        this->strType = string("Disengage");
        break;
    case 5:
        this->strType = string("Go park");
        break;
    }
    this->obj = obj;
    this->obj_init = obj;
    this->obj_eng = obj_eng;
    this->pose = posePtr(new Pose());

    this->prec=prec;
    if(prec){
        this->grip_str=string("Precision");
    }else{
        this->grip_str=string("Full");
    }
    /*
    if (prec){
        // precision grip
        switch (grip_id) {
        case 0:
            this->grip_code = 111; // Side thumb left
            this->grip_str=string("Precision Side thumb left");
            break;
        case 1:
            this->grip_code = 112; // Side thumb right
            this->grip_str=string("Precision Side thumb right");
            break;
        case 2:
            this->grip_code = 113; // Side thumb up
            this->grip_str=string("Precision Side thumb up");
            break;
        case 3:
            this->grip_code = 114; // Side thumb down
            this->grip_str=string("Precision Side thumb down");
            break;
        case 4:
            this->grip_code = 121; // Above
            this->grip_str=string("Precision Above");
            break;
        case 5:
            this->grip_code = 122; // Below
            this->grip_str=string("Precision Below");
            break;
        case 6:
            this->grip_code = 0;
            this->grip_str=string("No Grip");
            break;

        }
    }else{
        //full grip

        switch (grip_id) {
        case 0:
            this->grip_code = 211; // Side thumb left
            this->grip_str=string("Full Side thumb left");
            break;
        case 1:
            this->grip_code = 212; // Side thumb right
            this->grip_str=string("Full Side thumb right");
            break;
        case 2:
            this->grip_code = 213; // Side thumb up
            this->grip_str=string("Full Side thumb up");
            break;
        case 3:
            this->grip_code = 214; // Side thumb down
            this->grip_str=string("Full Side thumb down");
            break;
        case 4:
            this->grip_code = 221; // Above
            this->grip_str=string("Full Above");
            break;
        case 5:
            this->grip_code = 222; // Below
            this->grip_str=string("Full Below");
            break;
        case 6:
            this->grip_code = 0;
            this->grip_str=string("No Grip");
            break;

        }
    }
    */

    this->arm=arm;
    this->executed = false;

    this->obj_left=objectPtr(new Object());
    this->obj_init_left=objectPtr(new Object());
    this->obj_eng_left=objectPtr(new Object());
    this->pose_left = posePtr(new Pose());
    this->grip_str_left=string("No Grip");
    this->prec_left=false;


}

Movement::Movement(int type, int arm,objectPtr obj, objectPtr obj_eng, posePtr pose, bool prec)
{

    this->type=type;
    switch (type){
    case 0:
        this->strType = string("Reach-to-grasp");
        break;
    case 1:
        this->strType = string("Reaching");
        break;
    case 2:
        this->strType = string("Transport");
        break;
    case 3:
        this->strType = string("Engage");
        break;
    case 4:
        this->strType = string("Disengage");
        break;
    case 5:
        this->strType = string("Go park");
        break;
    }


    this->obj = obj;
    this->obj_init = obj;
    this->obj_eng = obj_eng;
    this->pose = pose;

    this->prec=prec;
    if(prec){
        this->grip_str=string("Precision");
    }else{
        this->grip_str=string("Full");
    }
    /*
    if (prec){
        // precision grip
        switch (grip_id) {
        case 0:
            this->grip_code = 111; // Side thumb left
            this->grip_str=string("Precision Side thumb left");
            break;
        case 1:
            this->grip_code = 112; // Side thumb right
            this->grip_str=string("Precision Side thumb right");
            break;
        case 2:
            this->grip_code = 113; // Side thumb up
            this->grip_str=string("Precision Side thumb up");
            break;
        case 3:
            this->grip_code = 114; // Side thumb down
            this->grip_str=string("Precision Side thumb down");
            break;
        case 4:
            this->grip_code = 121; // Above
            this->grip_str=string("Precision Above");
            break;
        case 5:
            this->grip_code = 122; // Below
            this->grip_str=string("Precision Below");
            break;
        case 6:
            this->grip_code = 0;
            this->grip_str=string("No Grip");
            break;

        }
    }else{
        //full grip

        switch (grip_id) {
        case 0:
            this->grip_code = 211; // Side thumb left
            this->grip_str=string("Full Side thumb left");
            break;
        case 1:
            this->grip_code = 212; // Side thumb right
            this->grip_str=string("Full Side thumb right");
            break;
        case 2:
            this->grip_code = 213; // Side thumb up
            this->grip_str=string("Full Side thumb up");
            break;
        case 3:
            this->grip_code = 214; // Side thumb down
            this->grip_str=string("Full Side thumb down");
            break;
        case 4:
            this->grip_code = 221; // Above
            this->grip_str=string("Full Above");
            break;
        case 5:
            this->grip_code = 222; // Below
            this->grip_str=string("Full Below");
            break;
        case 6:
            this->grip_code = 0;
            this->grip_str=string("No Grip");
            break;

        }
    }
    */

    this->arm=arm;
    this->executed = false;

    this->obj_left=objectPtr(new Object());
    this->obj_init_left=objectPtr(new Object());
    this->obj_eng_left=objectPtr(new Object());
    this->pose_left = posePtr(new Pose());
    this->grip_str_left=string("No Grip");
    this->prec_left=false;


}

Movement::Movement(int type, int arm, objectPtr obj, posePtr pose,bool prec)
{
    this->type=type;
    switch (type){
    case 0:
        this->strType = string("Reach-to-grasp");
        break;
    case 1:
        this->strType = string("Reaching");
        break;
    case 2:
        this->strType = string("Transport");
        break;
    case 3:
        this->strType = string("Engage");
        break;
    case 4:
        this->strType = string("Disengage");
        break;
    case 5:
        this->strType = string("Go park");
        break;
    }


    this->obj = obj;
    this->obj_init = obj;
    this->obj_eng = objectPtr(new Object());
    this->pose = pose;

    this->prec=prec;
    if(prec){
        this->grip_str=string("Precision");
    }else{
        this->grip_str=string("Full");
    }

    /*
    if (prec){
        // precision grip
        switch (grip_id) {
        case 0:
            this->grip_code = 111; // Side thumb left
            this->grip_str=string("Precision Side thumb left");
            break;
        case 1:
            this->grip_code = 112; // Side thumb right
            this->grip_str=string("Precision Side thumb right");
            break;
        case 2:
            this->grip_code = 113; // Side thumb up
            this->grip_str=string("Precision Side thumb up");
            break;
        case 3:
            this->grip_code = 114; // Side thumb down
            this->grip_str=string("Precision Side thumb down");
            break;
        case 4:
            this->grip_code = 121; // Above
            this->grip_str=string("Precision Above");
            break;
        case 5:
            this->grip_code = 122; // Below
            this->grip_str=string("Precision Below");
            break;
        case 6:
            this->grip_code = 0;
            this->grip_str=string("No Grip");
            break;

        }
    }else{
        //full grip

        switch (grip_id) {
        case 0:
            this->grip_code = 211; // Side thumb left
            this->grip_str=string("Full Side thumb left");
            break;
        case 1:
            this->grip_code = 212; // Side thumb right
            this->grip_str=string("Full Side thumb right");
            break;
        case 2:
            this->grip_code = 213; // Side thumb up
            this->grip_str=string("Full Side thumb up");
            break;
        case 3:
            this->grip_code = 214; // Side thumb down
            this->grip_str=string("Full Side thumb down");
            break;
        case 4:
            this->grip_code = 221; // Above
            this->grip_str=string("Full Above");
            break;
        case 5:
            this->grip_code = 222; // Below
            this->grip_str=string("Full Below");
            break;
        case 6:
            this->grip_code = 0;
            this->grip_str=string("No Grip");
            break;

        }
    }
    */

    this->arm=arm;
    this->executed = false;

    this->obj_left=objectPtr(new Object());
    this->obj_init_left=objectPtr(new Object());
    this->obj_eng_left=objectPtr(new Object());
    this->pose_left = posePtr(new Pose());
    this->grip_str_left=string("No Grip");
    this->prec_left=false;

}

Movement::Movement(int type_r,int type_l,int arm, objectPtr obj_r,bool prec_r,objectPtr obj_l,bool prec_l)
{
    // dual arm reach to grasp
    this->arm = arm;
    this->executed = false;

    this->obj = obj_r;
    this->obj_init = obj_r;

    this->obj_left = obj_l;
    this->obj_init_left = obj_l;

    this->prec=prec_r;
    if(prec_r){
        this->grip_str=string("Precision");
    }else{
        this->grip_str=string("Full");
    }

    this->prec_left=prec_l;
    if(prec_l){
        this->grip_str_left=string("Precision");
    }else{
        this->grip_str_left=string("Full");
    }

    this->type=type_r;
    switch (type){
    case 0:
        this->strType = string("Reach-to-grasp");
        break;
    case 1:
        this->strType = string("Reaching");
        break;
    case 2:
        this->strType = string("Transport");
        break;
    case 3:
        this->strType = string("Engage");
        break;
    case 4:
        this->strType = string("Disengage");
        break;
    case 5:
        this->strType = string("Go park");
        break;
    }

    this->type_left=type_l;
    switch (type_left){
    case 0:
        this->strType_left = string("Reach-to-grasp");
        break;
    case 1:
        this->strType_left = string("Reaching");
        break;
    case 2:
        this->strType_left = string("Transport");
        break;
    case 3:
        this->strType_left = string("Engage");
        break;
    case 4:
        this->strType_left = string("Disengage");
        break;
    case 5:
        this->strType_left = string("Go park");
        break;
    }

    this->obj_eng=objectPtr(new Object());
    this->pose = posePtr(new Pose());
    this->obj_eng_left=objectPtr(new Object());
    this->pose_left = posePtr(new Pose());

}

Movement::Movement(int type_r,int type_l,int arm)
{
    // dual arm reaching
    this->arm = arm;
    this->executed = false;

    this->type=type_r;
    switch (type){
    case 0:
        this->strType = string("Reach-to-grasp");
        break;
    case 1:
        this->strType = string("Reaching");
        break;
    case 2:
        this->strType = string("Transport");
        break;
    case 3:
        this->strType = string("Engage");
        break;
    case 4:
        this->strType = string("Disengage");
        break;
    case 5:
        this->strType = string("Go park");
        break;
    }
    this->type_left=type_l;
    switch (type_left){
    case 0:
        this->strType_left = string("Reach-to-grasp");
        break;
    case 1:
        this->strType_left = string("Reaching");
        break;
    case 2:
        this->strType_left = string("Transport");
        break;
    case 3:
        this->strType_left = string("Engage");
        break;
    case 4:
        this->strType_left = string("Disengage");
        break;
    case 5:
        this->strType_left = string("Go park");
        break;
    }

    this->prec=false;
    this->grip_str=string("No Grip");
    this->prec_left=false;
    this->grip_str_left=string("No Grip");

    this->obj = objectPtr(new Object());
    this->obj_init = objectPtr(new Object());
    this->obj_left = objectPtr(new Object());
    this->obj_init_left = objectPtr(new Object());
    this->obj_eng=objectPtr(new Object());
    this->pose = posePtr(new Pose());
    this->obj_eng_left=objectPtr(new Object());
    this->pose_left = posePtr(new Pose());
}

Movement::Movement(int type_r,int type_l,int arm, objectPtr obj_r,posePtr pose_r,bool prec_r,objectPtr obj_l,posePtr pose_l,bool prec_l)
{

    // dual arm transport
    this->arm = arm;
    this->executed = false;

    this->type=type_r;
    switch (type){
    case 0:
        this->strType = string("Reach-to-grasp");
        break;
    case 1:
        this->strType = string("Reaching");
        break;
    case 2:
        this->strType = string("Transport");
        break;
    case 3:
        this->strType = string("Engage");
        break;
    case 4:
        this->strType = string("Disengage");
        break;
    case 5:
        this->strType = string("Go park");
        break;
    }
    this->type_left=type_l;
    switch (type_left){
    case 0:
        this->strType_left = string("Reach-to-grasp");
        break;
    case 1:
        this->strType_left = string("Reaching");
        break;
    case 2:
        this->strType_left = string("Transport");
        break;
    case 3:
        this->strType_left = string("Engage");
        break;
    case 4:
        this->strType_left = string("Disengage");
        break;
    case 5:
        this->strType_left = string("Go park");
        break;
    }

    this->obj = obj_r;
    this->obj_init = obj_r;
    this->pose = pose_r;

    this->obj_left = obj_l;
    this->obj_init_left = obj_l;
    this->pose_left = pose_l;

    this->prec=prec_r;
    if(prec_r){
        this->grip_str=string("Precision");
    }else{
        this->grip_str=string("Full");
    }

    this->prec_left=prec_l;
    if(prec_l){
        this->grip_str_left=string("Precision");
    }else{
        this->grip_str_left=string("Full");
    }

    this->obj_eng=objectPtr(new Object());
    this->obj_eng_left=objectPtr(new Object());

}

Movement::Movement(int type_r,int type_l,int arm, objectPtr obj_r,objectPtr obj_eng_r,bool prec_r,objectPtr obj_l,objectPtr obj_eng_l,bool prec_l)
{
    // dual arm engage
    this->arm = arm;
    this->executed = false;

    this->type=type_r;
    switch (type){
    case 0:
        this->strType = string("Reach-to-grasp");
        break;
    case 1:
        this->strType = string("Reaching");
        break;
    case 2:
        this->strType = string("Transport");
        break;
    case 3:
        this->strType = string("Engage");
        break;
    case 4:
        this->strType = string("Disengage");
        break;
    case 5:
        this->strType = string("Go park");
        break;
    }
    this->type_left=type_l;
    switch (type_left){
    case 0:
        this->strType_left = string("Reach-to-grasp");
        break;
    case 1:
        this->strType_left = string("Reaching");
        break;
    case 2:
        this->strType_left = string("Transport");
        break;
    case 3:
        this->strType_left = string("Engage");
        break;
    case 4:
        this->strType_left = string("Disengage");
        break;
    case 5:
        this->strType_left = string("Go park");
        break;
    }

    this->obj = obj_r;
    this->obj_init = obj_r;
    this->obj_eng = obj_eng_r;

    this->obj_left = obj_l;
    this->obj_init_left = obj_l;
    this->obj_eng_left = obj_eng_l;

    this->prec=prec_r;
    if(prec_r){
        this->grip_str=string("Precision");
    }else{
        this->grip_str=string("Full");
    }

    this->prec_left=prec_l;
    if(prec_l){
        this->grip_str_left=string("Precision");
    }else{
        this->grip_str_left=string("Full");
    }

    this->pose=posePtr(new Pose());
    this->pose_left=posePtr(new Pose());
}


Movement::Movement(const Movement &mov)
{

    this->arm = mov.arm;
    this->type = mov.type;
    this->strType = mov.strType;
    this->type_left = mov.type_left;
    this->strType_left = mov.strType_left;
    //this->grip_code = mov.grip_code;
    this->prec=mov.prec;
    this->grip_str = mov.grip_str;

    this->obj = objectPtr(new Object(*mov.obj.get()));
    this->obj_init = objectPtr(new Object(*mov.obj_init.get()));
    this->obj_eng = objectPtr(new Object(*mov.obj_eng.get()));
    this->pose = posePtr(new Pose(*mov.pose.get()));
    this->executed = mov.executed;

    this->obj_left=objectPtr(new Object(*mov.obj_left.get()));
    this->obj_init_left=objectPtr(new Object(*mov.obj_init_left.get()));
    this->obj_eng_left=objectPtr(new Object(*mov.obj_eng_left.get()));
    this->pose_left = posePtr(new Pose(*mov.pose_left.get()));
    this->grip_str_left=mov.grip_str_left;
    this->prec_left=mov.prec_left;

}


Movement::~Movement()
{


}


void Movement::setType(int t)
{
    this->type=t;

    switch (type){

    case 0:

        this->strType = string("Reach-to-grasp");
        break;

    case 1:

        this->strType = string("Reaching");
        break;

    case 2:

        this->strType = string("Transport");
        break;

    case 3:

        this->strType = string("Engage");
        break;

    case 4:

        this->strType = string("Disengage");
        break;

    case 5:

        this->strType = string("Go park");
        break;


    }
}


void Movement::setTypeLeft(int t)
{
    this->type_left=t;

    switch (t){

    case 0:

        this->strType_left = string("Reach-to-grasp");
        break;

    case 1:

        this->strType_left = string("Reaching");
        break;

    case 2:

        this->strType_left = string("Transport");
        break;

    case 3:

        this->strType_left = string("Engage");
        break;

    case 4:

        this->strType_left = string("Disengage");
        break;

    case 5:

        this->strType_left = string("Go park");
        break;


    }
}



void Movement::setGrip(bool prec)
{

    this->prec=prec;
    if(prec){
        this->grip_str=string("Precision");
    }else{
        this->grip_str=string("Full");
    }
    /*
    this->grip_code=index;
    if (prec){
        // precision grip
        switch (index) {
        case 0:
            this->grip_code = 111; // Side thumb left
            this->grip_str=string("Precision Side thumb left");
            break;
        case 1:
            this->grip_code = 112; // Side thumb right
            this->grip_str=string("Precision Side thumb right");
            break;
        case 2:
            this->grip_code = 113; // Side thumb up
            this->grip_str=string("Precision Side thumb up");
            break;
        case 3:
            this->grip_code = 114; // Side thumb down
            this->grip_str=string("Precision Side thumb down");
            break;
        case 4:
            this->grip_code = 121; // Above
            this->grip_str=string("Precision Above");
            break;
        case 5:
            this->grip_code = 122; // Below
            this->grip_str=string("Precision Below");
            break;
        case 6:
            this->grip_code = 0;
            this->grip_str=string("No Grip");
            break;

        }
    }else{
        //full grip

        switch (index) {
        case 0:
            this->grip_code = 211; // Side thumb left
            this->grip_str=string("Full Side thumb left");
            break;
        case 1:
            this->grip_code = 212; // Side thumb right
            this->grip_str=string("Full Side thumb right");
            break;
        case 2:
            this->grip_code = 213; // Side thumb up
            this->grip_str=string("Full Side thumb up");
            break;
        case 3:
            this->grip_code = 214; // Side thumb down
            this->grip_str=string("Full Side thumb down");
            break;
        case 4:
            this->grip_code = 221; // Above
            this->grip_str=string("Full Above");
            break;
        case 5:
            this->grip_code = 222; // Below
            this->grip_str=string("Full Below");
            break;
        case 6:
            this->grip_code = 0;
            this->grip_str=string("No Grip");
            break;

        }
    }
    */
}

void Movement::setGripLeft(bool prec)
{

    this->prec_left=prec;
    if(prec){
        this->grip_str_left=string("Precision");
    }else{
        this->grip_str_left=string("Full");
    }

}


void Movement::setObject(objectPtr obj)
{
     this->obj=obj;
}

void Movement::setObjectLeft(objectPtr obj)
{
     this->obj_left=obj;
}


void Movement::setObjectInit(objectPtr obj)
{
     this->obj_init=obj;
}

void Movement::setObjectInitLeft(objectPtr obj)
{
     this->obj_init_left=obj;
}


void Movement::setObjectEng(objectPtr obj_eng)
{
   this->obj_eng = obj_eng;
}

void Movement::setObjectEngLeft(objectPtr obj_eng)
{
   this->obj_eng_left = obj_eng;
}


void Movement::setArm(int a)
{

    this->arm = a;
}


void Movement::setExecuted(bool exec)
{

    this->executed=exec;
}


int Movement::getType()
{

    return this->type;
}

int Movement::getTypeLeft()
{
    return this->type_left;
}

bool Movement::getGrip()
{
    return this->prec;
}

bool Movement::getGripLeft()
{
    return this->prec_left;
}

string Movement::getGripStr()
{
    return this->grip_str;
}

string Movement::getGripStrLeft()
{
    return this->grip_str_left;
}

objectPtr Movement::getObject()
{
    return obj;
}

objectPtr Movement::getObjectLeft()
{
    return obj_left;
}

posePtr Movement::getPose()
{
    return pose;
}

posePtr Movement::getPoseLeft()
{
    return pose_left;
}


objectPtr Movement::getObjectInit()
{
    return obj_init;
}

objectPtr Movement::getObjectInitLeft()
{
    return obj_init_left;
}

objectPtr Movement::getObjectEng()
{
    return obj_eng;
}

objectPtr Movement::getObjectEngLeft()
{
    return obj_eng_left;
}


string Movement::getStrType()
{
    return this->strType;
}

string Movement::getStrTypeLeft()
{
    return this->strType_left;
}


string Movement::getInfoLine()
{

    string arm_info;

    switch (arm) {
    case 0:
        arm_info = string("both");

        break;
    case 1:

        arm_info = string("right");
        break;

    case 2:
        arm_info = string("left");
        break;

    default:
        arm_info = string("right");
        break;
    }

    /*
    if (obj && obj_eng && grip_code!=0){

        return strType +", Arm: "+arm_info+", Object: "+obj->getName()+
                ", Object Engaged: "+obj_eng->getName()+", Grip Type: "+grip_str;

    }else if (obj && pose && grip_code!=0){

        return strType +", Arm: "+arm_info+", Object: "+obj->getName()+
                ", Pose: "+pose->getName()+", Grip Type: "+grip_str;

    }else if (obj && obj_eng && pose && grip_code!=0){

        return strType +", Arm: "+arm_info+", Object: "+obj->getName()+
                ", Object Engaged: "+obj_eng->getName() + ", Pose: "+pose->getName()+
                ", Grip Type: "+grip_str;

    }else if (pose){

        return strType +", Arm: "+arm_info+", Pose: "+pose->getName();

    }else if(obj && grip_code!=0){


        return strType +", Arm: "+arm_info+", Object: "+obj->getName()+
                ", Grip Type: "+grip_str;

    }else if(obj){

        return strType +", Arm: "+arm_info+", Object: "+obj->getName();

    }else{

        return strType+", Arm: "+arm_info;

    }
    */

    if(arm!=0)
    {
        // single arm
        return strType +", Arm: "+arm_info+", Object: "+obj->getName()+
                ", Object Engaged: "+obj_eng->getName() + ", Pose: "+pose->getName()+
                ", Grip Type: "+grip_str;
    }else{
        // dual arm
        string str_type = " ";
        if(this->type==this->type_left){
            str_type = strType;
        }
        return strType+", Arm: "+arm_info+", Object right: "+obj->getName()+", Object left: "+obj_left->getName()+
                ", Object right Engaged: "+obj_eng->getName() + ", Object left Engaged: "+obj_eng_left->getName()+
                ", Pose right: "+pose->getName()+", Pose left: "+pose_left->getName()+
                ", Grip Type right: "+grip_str+ ", Grip Type left: "+grip_str_left;
    }
}

int Movement::getArm()
{

    return this->arm;
}


bool Movement::getExecuted()
{

    return this->executed;
}


} // namespace motion_manager

