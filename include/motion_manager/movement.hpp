#ifndef MOVEMENT_HPP
#define MOVEMENT_HPP

#include "object.hpp"
#include "pose.hpp"

namespace motion_manager{

typedef boost::shared_ptr<Object> objectPtr;
typedef boost::shared_ptr<Pose> posePtr;

//! The Movement class
/**
 * @brief This class defines the movement of the upper-limbs of the humanoid robot
 */
class Movement
{

public:

    /**
     * @brief Movement, a constructor
     * @param type
     * @param arm
     */
    Movement(int type, int arm);

    /**
     * @brief Movement, a constructor
     * @param type
     * @param arm
     * @param obj
     */
    Movement(int type, int arm, objectPtr obj);

    /**
     * @brief Movement, a constructor
     * @param type
     * @param arm
     * @param pose
     */
    Movement(int type, int arm, posePtr pose);

    /**
     * @brief Movement
     * @param type
     * @param arm
     * @param obj
     * @param prec
     */
    Movement(int type, int arm, objectPtr obj,bool prec);
    //Movement(int type, int arm, objectPtr obj, int grip_id, bool prec);

    /**
     * @brief Movement
     * @param type_r
     * @param type_l
     * @param arm
     * @param obj_r
     * @param prec_r
     * @param obj_l
     * @param prec_l
     */
    Movement(int type_r,int type_l,int arm, objectPtr obj_r,bool prec_r,objectPtr obj_l,bool prec_l);

    /**
     * @brief Movement
     * @param type_r
     * @param type_l
     * @param arm
     */
    Movement(int type_r,int type_l,int arm);

    /**
     * @brief Movement
     * @param type_r
     * @param type_l
     * @param arm
     * @param obj_r
     * @param pose_r
     * @param prec_r
     * @param obj_l
     * @param pose_l
     * @param prec_l
     */
    Movement(int type_r,int type_l,int arm, objectPtr obj_r,posePtr pose_r,bool prec_r,objectPtr obj_l,posePtr pose_l,bool prec_l);

    /**
     * @brief Movement
     * @param type_r
     * @param type_l
     * @param arm
     * @param obj_r
     * @param obj_eng_r
     * @param prec_r
     * @param obj_l
     * @param obj_eng_l
     * @param prec_l
     */
    Movement(int type_r, int type_l, int arm, objectPtr obj_r, objectPtr obj_eng_r, bool prec_r, objectPtr obj_l, objectPtr obj_eng_l, bool prec_l);

    /**
     * @brief Movement
     * @param type
     * @param arm
     * @param obj
     * @param obj_eng
     * @param prec
     */
    Movement(int type, int arm, objectPtr obj, objectPtr obj_eng, bool prec);
    //Movement(int type, int arm, objectPtr obj, objectPtr obj_eng, int grip_id, bool prec);

    /**
     * @brief Movement
     * @param type
     * @param arm
     * @param obj
     * @param obj_eng
     * @param pose
     * @param prec
     */
    Movement(int type, int arm, objectPtr obj, objectPtr obj_eng, posePtr pose, bool prec);
    //Movement(int type, int arm, objectPtr obj, objectPtr obj_eng, posePtr pose,int grip_id, bool prec);

    /**
     * @brief Movement
     * @param type
     * @param arm
     * @param obj
     * @param pose
     * @param prec
     */
    Movement(int type, int arm, objectPtr obj, posePtr pose,bool prec);
    //Movement(int type, int arm, objectPtr obj, posePtr pose, int grip_id, bool prec);


    /**
     * @brief Movement, a copy constructor
     * @param mov
     */
    Movement(const Movement& mov);

    /**
     * @brief ~Movement, a destructor
     */
    ~Movement();


    /**
    * @brief This method sets the type of the movement (right).
    *  @param t
    */
    void setType(int t);

    /**
    * @brief This method sets the type of the movement (left).
    *  @param t
    */
    void setTypeLeft(int t);

    /**
     * @brief This method sets the type of grip
     * related to the movement (right).
     * @param prec
     */
    void setGrip(bool prec);

    /**
     * @brief This method sets the type of grip
     * related to the movement (left).
     * @param prec
     */
    void setGripLeft(bool prec);

    /**
     * @brief This method sets the object that is manipulated
     * during the movement (right)
     * @param obj
     */
    void setObject(objectPtr obj);

    /**
     * @brief This method sets the object that is manipulated
     * during the movement (left)
     * @param obj
     */
    void setObjectLeft(objectPtr obj);

    /**
     * @brief This method sets the object that is manipulated
     * before the movement starts (right)
     * @param obj
     */
    void setObjectInit(objectPtr obj);

    /**
     * @brief This method sets the object that is manipulated
     * before the movement starts (left)
     * @param obj
     */
    void setObjectInitLeft(objectPtr obj);

    /**
     * @brief This method sets the object that is involved
     * in engaging/disengaging movements (right)
     * @param obj_eng
     */
    void setObjectEng(objectPtr obj_eng);

    /**
     * @brief This method sets the object that is involved
     * in engaging/disengaging movements (left)
     * @param obj_eng
     */
    void setObjectEngLeft(objectPtr obj_eng);

    /**
     * @brief This method sets the arm/s in movement
     * @param a
     */
    void setArm(int a);

    /**
     * @brief This method gets the the status of the movement
     * @param exec
     */
    void setExecuted(bool exec);

    /**
     * @brief This method gets the code of the movement (right)
     * @return
     */
    int getType();

    /**
     * @brief This method gets the code of the movement (left)
     * @return
     */
    int getTypeLeft();

    /**
     * @brief This method gets the description of the movement (right)
     * @return
     */
    string getStrType();

    /**
     * @brief This method gets the description of the movement (left)
     * @return
     */
    string getStrTypeLeft();

    /**
     * @brief This method gets the type of the grip (right)
     * @return
     */
    bool getGrip();

    /**
     * @brief This method gets the type of the grip (left)
     * @return
     */
    bool getGripLeft();

    /**
     * @brief This method gets the description of the grip (right)
     * @return
     */
    string getGripStr();

    /**
     * @brief This method gets the description of the grip (left)
     * @return
     */
    string getGripStrLeft();

    /**
     * @brief This method gets the object that is being manipulated
     * during the movement (right)
     * @return
     */
    objectPtr getObject();

    /**
     * @brief This method gets the object that is being manipulated
     * during the dual-arm movement by the left hand
     * @return
     */
    objectPtr getObjectLeft();

    /**
     * @brief This method gets the pose during the movement (right)
     * @return
     */
    posePtr getPose();

    /**
     * @brief This method gets the pose during the movement (left)
     * @return
     */
    posePtr getPoseLeft();

    /**
     * @brief This method gets the object that is being manipulated
     * before the movement starts (right)
     * @return
     */
    objectPtr getObjectInit();

    /**
     * @brief This method gets the object that is being manipulated
     * before the movement starts (left)
     * @return
     */
    objectPtr getObjectInitLeft();

    /**
     * @brief This method gets the object that is involved
     * in engaging/disengaging movements (right)
     * @return
     */
    objectPtr getObjectEng();

    /**
     * @brief This method gets the object that is involved
     * in engaging/disengaging movements (left)
     * @return
     */
    objectPtr getObjectEngLeft();

    /**
     * @brief This method gets information about the movement
     * @return
     */
    string getInfoLine();

    /**
     * @brief This method gets the arm/s in movement
     * @return
     */
    int getArm();

    /**
     * @brief This method gets the the status of the movement
     * @return
     */
    bool getExecuted();

private:
    int arm;  /**< 0 = both arms, 1 = right-arm, 2 = left-arm */

   /**
     * @brief type of the movement (code)
     * <table>
     * <caption id="multi_row">Types of the movement</caption>
     * <tr><th>Type      <th>Code
     * <tr><td>Reach-to-grasp <td>0
     * <tr><td>Reaching <td>1
     * <tr><td>Transport <td>2
     * <tr><td>Engage <td>3
     * <tr><td>Disengage <td>4
     * <tr><td>Go park <td>5
     * </table>
     */
    int type;
    string strType; /**< type of the movement (string) */
    int type_left;
    string strType_left; /**< type of the movement (string) */
    bool executed; /**< true if the movement has been executed, false otherwise */

    /**
     * @brief type of the grip (code)
     * <table>
     * <caption id="multi_row">Types of grip</caption>
     * <tr><th>Type      <th>Precision (prec = true) <th>Full (prec = false) <th> Index
     * <tr><td>Side thumb left <td>111 <td>211 <td>0
     * <tr><td>Side thumb right <td>112 <td>212 <td>1
     * <tr><td>Side thumb up <td>113 <td>213 <td>2
     * <tr><td>Side thumb down <td>114 <td>214 <td>3
     * <tr><td>Above <td>121 <td>221 <td>4
     * <tr><td>Below <td>122 <td>222 <td>5
     * <tr><td>No Grip <td>- <td>- <td>6
     * </table>
     */
    //int grip_code;
    bool prec; // true for a precision grip, false otherwise
    string grip_str; /**< type of the grip (string) */
    objectPtr obj; /**< object being manipulated in the movement */
    objectPtr obj_init; /**< object being manipulated before the movement starts */
    objectPtr obj_eng; /**< object involved in engaging/disengaging movements */
    posePtr pose; /**< pose involved in transport/disengaging/reaching movements*/


    // dual arm members
    bool prec_left; // true for a precision grip, false otherwise
    string grip_str_left; /**< type of the grip (string) */
    objectPtr obj_left; /**< object being manipulated in the movement */
    objectPtr obj_init_left; /**< object being manipulated before the movement starts */
    objectPtr obj_eng_left; /**< object involved in engaging/disengaging movements */
    posePtr pose_left; /**< pose involved in transport/disengaging/reaching movements*/


    // Types of movements
    // ||||||||||||||||||||||||||
    // |||||||||||||||||| Code ||
    // ||Reach-to-grasp |   0  ||
    // ||Reaching       |   1  ||
    // ||Transport      |   2  ||
    // ||Engage         |   3  ||
    // ||Disengage      |   4  ||
    // ||Go park        |   5  ||
    // ||||||||||||||||||||||||||

    // Types of Grip and related code
    // |||||||||||||||||||||||||||||||||||||||||||||||||
    // |||||||||||||||||||| Precision |  Full | Index ||
    // ||Side thumb left  |    111    |  211  |   0   ||
    // ||Side thumb right |    112    |  212  |   1   ||
    // ||Side thumb up    |    113    |  213  |   2   ||
    // ||Side thumb down  |    114    |  214  |   3   ||
    // ||Above            |    121    |  221  |   4   ||
    // ||Below            |    122    |  222  |   5   ||
    // ||No Grip          |     -     |   -   |   6   ||
    // |||||||||||||||||||||||||||||||||||||||||||||||||


};

} // namespace motion_manager

#endif // MOVEMENT_HPP
