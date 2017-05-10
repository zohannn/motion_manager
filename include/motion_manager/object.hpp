#ifndef OBJECT_HPP
#define OBJECT_HPP

#include "target.hpp"
#include "engagepoint.hpp"

namespace motion_manager{

typedef boost::shared_ptr<Target> targetPtr; /**< shared pointer to a target */
typedef boost::shared_ptr<EngagePoint> engagePtr; /**< shared pointer to a engage point */

//! The Object class
/**
 * @brief This class defines the concept of a generic object in the scenario
 */
class Object: public Point
{

public:
    /**
     * @brief Object, a constructor
     */
    Object();

    /**
     * @brief Object, a constructor
     * @param name
     */
    explicit Object(string name);

    /**
     * @brief Object, a constructor
     * @param name
     * @param ppos
     * @param oor
     * @param ssize
     * @param pTR
     * @param pTL
     * @param pEng
     */
    Object(string name, pos ppos, orient oor, dim ssize,
                    Target* pTR, Target* pTL,
                    EngagePoint* pEng);

    /**
     * @brief Object, a copy constructor
     * @param obj
     */
    Object(const Object& obj);

    /**
     * @brief ~Object, a destructor
     */
    ~Object();


    /**
     * @brief This method sets the position of the object.\n
     * If update_features is true, then the related features of the objects will be updated.
     * @param ppos
     * @param update_features
     */
    void setPos(pos& ppos, bool update_features);

    /**
     * @brief This method sets the orientation of the object.\n
     * If update_features is true, then the related features of the objects will be updated.
     * @param oor
     * @param update_features
     */
    void setOr(orient& oor, bool update_features);

    /**
     * @brief This method sets the size of the object.
     * @param ssize
     */
    void setSize(dim ssize);

    /**
     * @brief This method sets the handle of the object
     * @param h
     */
    void setHandle(int h);

    /**
     * @brief This method sets tha handle of the body of the object
     * @param h
     */
    void setHandleBody(int h);

    /**
     * @brief This method sets the target of the right arm
     * @param tr
     * @return
     */
    bool setTargetRight(targetPtr tr);

    /**
     * @brief This method sets the target of the left arm
     * @param tl
     * @return
     */
    bool setTargetLeft(targetPtr tl);

    /**
     * @brief This method enables/disables the target of the right arm
     * @param c
     */
    void setTargetRightEnabled(bool c);

    /**
     * @brief This method enables/disables the target of the left arm
     * @param c
     */
    void setTargetLeftEnabled(bool c);

    /**
     * @brief This method sets the engage point
     * @param eng
     * @return
     */
    bool setEngagePoint(engagePtr eng);

    /**
     * @brief This method gets the size of the object
     * @return
     */
    dim getSize();

    /**
     * @brief This method gets the handle of the object
     * @return
     */
    int getHandle();

    /**
     * @brief This method gets the handle of the body of the object
     * @return
     */
    int getHandleBody();

    /**
     * @brief This method gets the target of the right arm
     * @return
     */
    targetPtr getTargetRight();

    /**
     * @brief This method gets the target of the left arm
     * @return
     */
    targetPtr getTargetLeft();

    /**
     * @brief This method gets the engage point
     * @return
     */
    engagePtr getEngagePoint();

    /**
     * @brief This method gets true if the target of the right arm is enabled, false otherwise
     * @return
     */
    bool isTargetRightEnabled();

    /**
     * @brief This method gets true if the target of the left arm is enabled, false otherwise
     * @return
     */
    bool isTargetLeftEnabled();

    /**
     * @brief This method gets the radius of the object as the maximum radius on the XY plane
     * @return
     */
    double getRadius();

    /**
     * @brief This method gets information about the object
     * @return
     */
    string getInfoLine();

    /**
     * @brief getEngTarRight
     * @param eng_to_tar
     */
    void getEngTarRight(std::vector<double>& eng_to_tar);

    /**
     * @brief getEngTarLeft
     * @param eng_to_tar
     */
    void getEngTarLeft(std::vector<double>& eng_to_tar);

    /**
     * @brief getTarRightObj
     * @param tar_to_obj
     */
    void getTarRightObj(std::vector<double>& tar_to_obj);

    /**
     * @brief getTarLeftObj
     * @param tar_to_obj
     */
    void getTarLeftObj(std::vector<double>& tar_to_obj);

    /**
     * @brief getEngObj
     * @param eng_to_obj
     */
    void getEngObj(std::vector<double>& eng_to_obj);



private:


    dim m_size; /**< size of the object */
    int handle; /**< handle of the object */
    int handle_body; /**< handle of the visible part of the object (the body) */
    bool m_targetRightEnabled; /**< true if the object is a target of the right arm, false otherwise */
    bool m_targetLeftEnabled; /**< true if the object is a target of the left arm, false otherwise */
    targetPtr p_targetRight; /**< target of the right arm */
    targetPtr p_targetLeft; /**< target of the left arm */
    engagePtr p_engage; /**< engage point of the object */
    std::vector<double> eng_tar_right; /**< position of the engage point with respect to the target right in the target right frame. x=0, y=1, z=2*/
    std::vector<double> eng_tar_left; /**< position of the engage point with respect to the target left in the target left frame. x=0, y=1, z=2*/
    std::vector<double> eng_obj; /**< position of the engage point with respect to the object center in the object frame. x=0, y=1, z=2*/
    std::vector<double> tar_right_obj; /**< position of the target right with respect to the object in the object frame. x=0, y=1, z=2 */
    std::vector<double> tar_left_obj; /**< position of the target left with respect to the object in the object frame. x=0, y=1, z=2 */
    bool setup_features; /**< true if it is necessary to set up the features, false otherwise (the object might be created with no features)*/

    /**
     * @brief This method gets the transformation matrix of the target of the right arm
     * @param mat
     */
    void getTar_right_matrix(Matrix4d& mat);

    /**
     * @brief This method gets the transformation matrix of the target of the left arm
     * @param mat
     */
    void getTar_left_matrix(Matrix4d& mat);

    /**
     * @brief This method gets the transformation matrix of the engage point
     * @param mat
     */
    void getEngage_matrix(Matrix4d& mat);

    /**
     * @brief Get the orientation in Roll-Pitch-Yaw from the transformation matrix
     * @param Trans
     * @param rpy
     */
    void getRPY(Matrix4d Trans, vector<double>& rpy);



};

} // namespace motion_manager

#endif // OBJECT_HPP
