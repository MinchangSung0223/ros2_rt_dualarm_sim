#ifndef MR_INDY7_DUALARM_H
#define MR_INDY7_DUALARM_H
#include <jsoncpp/json/json.h>
#include "iostream"
#include "MR_Indy7.h"

#include "modern_robotics.h"
#include "modern_robotics_relative.h"

#pragma comment(lib, "jsoncpp.lib")
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
using namespace mr;
#define RELJOINTNUM 12
class MR_Indy7_DualArm {
public:
    MR_Indy7_DualArm();  // Constructor
    typedef Eigen::Matrix<double, RELJOINTNUM, 1> relJVec;
    typedef Eigen::Matrix<double, 6, RELJOINTNUM> relScrewList;
    MR_Indy7* L;
    MR_Indy7* R;
    unsigned int jointnum;
    relmr::ScrewList Slist;
    relmr::ScrewList Blist;

    mr::SE3 M;
    mr::SE3 Tbl;

    relJVec q;
    relJVec dq;
    relJVec ddq;


    relJVec q_des;
    relJVec dq_des;
    relJVec ddq_des;

    mr::Vector3d g;
    mr::JVec  torq;

    mr::Matrix6d Kp;
    mr::Matrix6d Kv;
    mr::Matrix6d Ki;

    mr::Matrix6d Hinf_Kp;
    mr::Matrix6d Hinf_Kv;
    mr::Matrix6d Hinf_Ki;
    mr::Matrix6d Hinf_K_gamma;

    void MRSetup();
    void setq(mr::JVec q_l, mr::JVec q_r);
    void setdq(mr::JVec dq_l, mr::JVec dq_r);
    void getq(mr::JVec& q_l, mr::JVec& q_r);
    void getdq(mr::JVec& dq_l, mr::JVec& dq_r);
    relmr::JVec GravityForces(const relmr::JVec q);
    relmr::JVec VelQuadraticForces(const relmr::JVec q,const relmr::JVec qdot);
    relmr::MassMat MassMatrix(const relmr::JVec q);
    relmr::JVec ForwardDynamics(const relmr::JVec q,const relmr::JVec qdot,const relmr::JVec tau,const Vector6d Ftip_r,const Vector6d Ftip_l);

};

#endif // MR_INDY7_DUALARM_H
