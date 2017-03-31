#ifndef SCENARIO_HPP
#define SCENARIO_HPP

#include "humanoid.hpp"
#include "object.hpp"
#include "pose.hpp"

namespace motion_manager{

typedef boost::shared_ptr<Humanoid> humanoidPtr; /**< shared pointer to a humanoid */
typedef boost::shared_ptr<Object> objectPtr; /**< shared pointer to an object*/
typedef boost::shared_ptr<Pose> posePtr; /**< shared pointer to a pose*/

//! The Scenario Class
/**
 * @brief This class defines the concept of scenario where the robot and/or a human partner is acting
 */
class Scenario
{

public:

    /**
     * @brief Scenario, a constructor
     * @param name
     * @param id
     */
    Scenario(string name, int id);

    /**
     * @brief Scenario, a copy constructor
     * @param scene
     */
    Scenario(const Scenario& scene);


    /**
     * @brief ~Scenario, a destructor.
     */
    ~Scenario();


    /**
     * @brief This method sets the name of the scenario
     * @param name
     */
    void setName(string& name);

    /**
     * @brief This method sets the ID of the scenario
     * @param id
     */
    void setID(int id);

    /**
     * @brief This method insert an object in the vector of objects at the position pos
     * @param pos
     * @param obj
     */
    void setObject(int pos, objectPtr obj);

    /**
     * @brief This method insert a pose in the vector of poses at the position pos
     * @param pos
     * @param pt
     */
    void setPose(int pos, posePtr pt);

    /**
     * @brief This method gets the name of the scenario
     * @return
     */
    string getName();

    /**
     * @brief This method gets the ID of the scenario
     * @return
     */
    int getID();

    /**
     * @brief This method gets a pointer to the humanoid robot in the scenario
     * @return
     */
    humanoidPtr getHumanoid();

    /**
     * @brief This method get the list of the objects in the scenario
     * @param objs
     * @return
     */
    bool getObjects(vector<objectPtr>& objs);

    /**
     * @brief This method get the list of the poses in the scenario
     * @param pts
     * @return
     */
    bool getPoses(vector<posePtr>& pts);

    /**
     * @brief This method adds a new object to the scenario
     * @param obj_ptr
     */
    void addObject(objectPtr obj_ptr);

    /**
     * @brief addPose
     * @param pose_ptr
     */
    void addPose(posePtr pose_ptr);

    /**
     * @brief This method gets the object at the position pos in the vector of objects
     * @param pos
     * @return
     */
    objectPtr getObject(int pos);

    /**
     * @brief getPose
     * @param pos
     * @return
     */
    posePtr getPose(int pos);

    /**
     * @brief This method gets the object named "obj_name"
     * @param obj_name
     * @return
     */
    objectPtr getObject(string obj_name);

    /**
     * @brief getPose
     * @param pose_name
     * @return
     */
    posePtr getPose(string pose_name);

    /**
     * @brief This method adds the humanoid robot to the scenario
     * @param hh_ptr
     */
    void addHumanoid(humanoidPtr hh_ptr);


private:

    string m_name; /**< the name of the scenario*/
    int m_scenarioID; /**< the ID of the scenario*/
    vector<objectPtr> objs_list; /**< the objects in the scenario */
    vector<posePtr> poses_list; /**< the poses in the scenario */
    humanoidPtr hPtr; /**< the humanoid robot in the scenario */



};

} // namespace motion_manager


#endif // SCENARIO_HPP
