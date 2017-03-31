#ifndef POSE_HPP
#define POSE_HPP

#include "point.hpp"

namespace motion_manager{

//! The Pose class
/**
 * @brief This class defines the pose that is taken into account for the constraints on the hand.
 * The pose is a point designed to be used either as reference to place an object or as target that os not related to any object.
 */
class Pose:public Point
{
public:
    /**
     * @brief Pose, default constructor.
     */
    Pose();

    /**
     * @brief Pose, a constructor.
     * @param name
     * @param ppos
     * @param oor
     */
    Pose(string name, pos ppos, orient oor);

    /**
     * @brief Pose, a constructor
     * @param name
     * @param ppos
     * @param oor
     */
    Pose(string name, pos ppos, orient oor, bool obj_rel, int id);

    /**
     * @brief Pose, a copy constructor.
     * @param tar
     */
    Pose(const Pose& tar);


    /**
     * @brief ~Pose, a destructor.
     */
    ~Pose();

    /**
     * @brief setObjRel
     * @param obj_rel
     */
    void setObjRel(bool obj_rel);

    /**
     * @brief setObjId
     * @param id
     */
    void setObjId(int id);

    /**
     * @brief getObjRel
     * @return
     */
    bool getObjRel();

    /**
     * @brief getObjId
     * @return
     */
    int getObjId();

private:

    bool obj_related; /**< true if the pose is related to an object (for place movements), false otherwise (for move movements) */
    int obj_id; /**< id of the object to which the pose is related to, -1 otherwise */


};
}// namespace motion_manager

#endif // POSE_HPP
