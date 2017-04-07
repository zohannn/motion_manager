#ifndef COMMON_HPP
#define COMMON_HPP

#include <string>
#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <list>
#include <vector>
#include <cmath>
#include <boost/smart_ptr.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/format.hpp>
#include <fstream>
#include <eigen3/Eigen/Dense>

/** configuration */
#include "config.hpp"

/** definitions for printing output */
#define SEP ", "
#define METERS " [m]"
#define MILLIMETERS " [mm]"
#define DEG " [deg]"
#define RAD " [rad]"
#define COLUMN ":"
#define SPACE " "
#define XposSTR "Xpos = "
#define YposSTR "Ypos = "
#define ZposSTR "Zpos = "
#define RollSTR "Roll = "
#define PitchSTR "Pitch = "
#define YawSTR "Yaw = "
#define XsizeSTR "Xsize = "
#define YsizeSTR "Ysize = "
#define ZsizeSTR "Zsize = "

//definition of the macro ASSERT
#ifndef DEBUG
    #define ASSERT(x)
#else
    #define ASSERT(x) \
             if (! (x)) \
            { \
               cout << "ERROR!! Assert " << #x << " failed\n"; \
               cout << " on line " << __LINE__  << "\n"; \
               cout << " in file " << __FILE__ << "\n";  \
            }
#endif

using namespace std;
using namespace Eigen;

namespace motion_manager{

#if HAND==0
    // Human hand
    const double THETA8_HOME = 0.0; /**< constant of the joint 8 (spread of the hand) in [rad] */
    const double THETA8_FINAL = -double(M_PI)/2; /**< constant of the joint 8 (spread of the hand) in [rad] */
    const double TOL_GRIP = 5.0; /**< tolerance on the grip in [mm] */
#elif HAND==1
    // Barrett hand
    const double THETA8_HOME = 0.0; /**< constant of the joint 8 (spread of the hand) in [rad] */
    const double THETA8_FINAL = 0.0; /**< constant of the joint 8 (spread of the hand) in [rad] */
    const double TOL_GRIP = 10.0; /**< tolerance on the grip in [mm] */
    const int firstPartTorqueOvershootCountRequired = 1;/**< number of time that the torque applied to the first phalanx is bigger than firstPartMaxTorque */
    const double firstPartMaxTorque = 0.9f;/**< max torque that can be applied to the first phalanx of the fingers */
    const double closingOpeningTorque = 1.0f;/**< torque applied to the fingers when they are opening/closing */
    const double closingVel = 60.0f * static_cast<double>(M_PI) / 180.0f; /**< joint velocity of the fingers when they are closing */
    const double openingVel = -120.0f * static_cast<double>(M_PI) / 180.0f;/**< joint velocity of the fingers when they are opening */
#endif

const int HAND_FINGERS = 3; /**< number of fingers per hand */
const int JOINTS_ARM = 7; /**< number of joints per arm */
const int JOINTS_HAND = 4; /**< number of joints per hand */
const int N_PHALANGE = 3; /**< number of phalanges per finger */

/** this struct defines the position in the Cartesian space*/
typedef struct{
    double Xpos; /**< position along the x axis in [mm] */
    double Ypos; /**< position along the y axis in [mm] */
    double Zpos; /**< position along the z axis in [mm] */
} pos;

/** this struct defines the orientation in Roll-Pitch-Yaw */
typedef struct{
    double roll; /**< rotarion around the z axis in [rad] */
    double pitch; /**< rotarion around the y axis in [rad] */
    double yaw; /**< rotarion around the x axis in [rad] */
} orient;

/** this struct defines the dimention of an object */
typedef struct{
    double Xsize; /**< size of the object along the x axis in [mm] */
    double Ysize; /**< size of the object along the y axis in [mm] */
    double Zsize; /**< size of the object along the z axis in [mm] */
} dim;

/** this struct defines the Denavit-Hartenberg kinematic parameters */
typedef struct{
    vector<double> d; /**< distances between consecutive frames along the y axes in [mm] */
    vector<double> a; /**< distances between concecutive frames along the z axes in [mm] */
    vector<double> alpha; /**< angle around the x axes between consecutive z axes in [rad] */
    vector<double> theta; /**< angle around the z axes between consecutive x axes in [rad] */
} DHparams;

/** this struct defines the arm */
typedef struct{
    DHparams arm_specs; /**< the Denavit-Hartenberg parameters of the arm */
} arm;

/** this struct defines the barrett hand */
typedef struct{
    double maxAperture; /**< [mm] max aperture of the hand in [mm] */
    double Aw; /**< smallest distance between F1 and F2 in [mm] */
    double A1; /**< length of the 1st part of the finger in [mm] */
    double A2; /**< length of the first phalax in [mm] */
    double A3; /**< length of the second phalax in [mm] */
    double D3; /**< depth of the fingertip in [mm] */
    double phi2; /**< angular displacement between the 1st part of the finger and the 1st phalax in [rad] */
    double phi3; /**< angular displacement between the 1st and the 2nd phalax in [rad] */
} barrett_hand;

/** this struct defines a human finger */
typedef struct{
    double ux; /**<  position of the finger with respect to the center of the palm along the x axis in [mm] */
    double uy; /**<  position of the finger with respect to the center of the palm along the y axis in [mm] */
    double uz; /**<  position of the finger with respect to the center of the palm along the z axis in [mm] */
    DHparams finger_specs; /**< the Denavit-Hartenberg parameters of the finger */
} human_finger;

/** this struct defines a human thumb */
typedef struct{
    double uTx; /**<  position of the thumb with respect to the center of the palm along the x axis in [mm] */
    double uTy; /**<  position of the thumb with respect to the center of the palm along the y axis in [mm] */
    double uTz; /**<  position of the thumb with respect to the center of the palm along the z axis in [mm] */
    DHparams thumb_specs; /**< the Denavit-Hartenberg parameters of the thumb */
} human_thumb;

/** this struct defines a human hand */
typedef struct{
  vector<human_finger> fingers; /**< fingers of the hand */
  human_thumb thumb; /**<  thumb of the hand */
  double maxAperture; /**< max aperture of the hand in [mm] */
} human_hand;

/** this struct defines a generic part of a humanoid body */
typedef struct{
    double Xpos; /**< position of the part along the x axis in [mm] */
    double Ypos; /**< position of the part along the y axis in [mm] */
    double Zpos; /**< position of the part along the z axis in [mm] */
    double Roll; /**< orientation of the part around the z axis in [rad] */
    double Pitch; /**< orientation of the part around the y axis in [rad] */
    double Yaw; /**< orientation of the part around the x axis in [rad] */
    double Xsize; /**< size of the part along the x axis in [mm] */
    double Ysize; /**< size of the part along the y axis in [mm] */
    double Zsize; /**< size of the part along the z axis in [mm] */
} humanoid_part;



}// namespace motion_manager

#endif // COMMON_HPP
