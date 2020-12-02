#ifndef HBINVERSE
#define HBINVERSE
#define         PI          3.141592

#include "BasicMatrix.h"

using namespace std;

struct ArmJoints
{
    union{
        struct{
            double RSP,RSR,RSY,REB,RWY,RWP,RF1;
            double LSP,LSR,LSY,LEB,LWY,LWP,LF1;
        };
        double AJ_Array[14];
    };
    double WST;

    vec3 pPel;
    quat qPel;

};
struct LegJoints
{
    union{
        struct{
            double RHY,RHR,RHP,RKN,RAP,RAR;
            double LHY,LHR,LHP,LKN,LAP,LAR;
        };
        double LJ_Array[12];
    };

    vec3 pPel;
    quat qPel;
    double WST;
};
struct FeetPos
{
  vec3 pRF;
  quat qRF;
  vec3 pLF;
  quat qLF;
  vec3 pCOM;
  vec3 pPel;
  quat qPel;
  double pPelz;
};

struct ArmPos
{
    vec3 pRH;
    quat qRH;
    vec3 pLH;
    quat qLH;
    vec3 pCOM;
    vec3 pPel;
    quat qPel;
};

class HB_inverse
{
public:
    double err_max;

    double m_rhy;
    double m_rhr;
    double m_rhp;
    double m_rkn;
    double m_rap;
    double m_rar;
    double m_rleg;

    double m_lhy;
    double m_lhr;
    double m_lhp;
    double m_lkn;
    double m_lap;
    double m_lar;
    double m_lleg;

    double m_rsp;
    double m_rsr;
    double m_rsy;
    double m_reb;
    double m_rwy;
    double m_rwp;
    double m_rf1;

    double m_lsp;
    double m_lsr;
    double m_lsy;
    double m_leb;
    double m_lwy;
    double m_lwp;
    double m_lf1;

    double m_torso;
    double m_pel;
    double m_total;

    //double m_ub;

    double P2HR;   // l1
    double HR2HPy; // l2
    double HR2HPz; // l3
    double ULEG;   // l4
    double LLEG;   // l5
    double A2F;    // l6

    // for drc IK pel
    double P2H;
    double AP2AR;

    vec3 offset_p2rh, offset_p2lh, offset_ankle;


    double P2SC;
    double SC2S;
    double UARM;
    double LARM;
    double OFFELB;

    vec3 offset_p2rHR, offset_p2lHR, offset_rHR2rHP, offset_lHR2lHP, offset_uleg, offset_lleg, offset_foot;
    vec3 c_rhy, c_rhr, c_rhp, c_rkn, c_rap, c_rar;
    vec3 c_lhy, c_lhr, c_lhp, c_lkn, c_lap, c_lar;
    mat3 I_rhy, I_rhr, I_rhp, I_rkn, I_rap, I_rar;
    mat3 I_lhy, I_lhr, I_lhp, I_lkn, I_lap, I_lar;

    vec3 offset_p2s_center, offset_s_center2rs, offset_s_center2ls, offset_uarm, offset_larm, offset_elbow;
    vec3 c_rsp, c_rsr, c_rsy, c_reb, c_rwy, c_rwp, c_rf1;
    vec3 c_lsp, c_lsr, c_lsy, c_leb, c_lwy, c_lwp, c_lf1;
    mat3 I_rsp, I_rsr, I_rsy, I_reb, I_rwy, I_rwp, I_rf1;
    mat3 I_lsp, I_lsr, I_lsy, I_leb, I_lwy, I_lwp, I_lf1;


    vec3 c_torso, c_pel;
    mat3 I_torso, I_pel;

    int QT2DC(const double qt_4x1[], double DC_3x3[])  // convert a quaternion to a direction cosine matrix
    {
        double temp = sqrtp(qt_4x1[0]*qt_4x1[0] + qt_4x1[1]*qt_4x1[1]
                            + qt_4x1[2]*qt_4x1[2] + qt_4x1[3]*qt_4x1[3]);
        double q0 = qt_4x1[0]/temp;
        double q1 = qt_4x1[1]/temp;
        double q2 = qt_4x1[2]/temp;
        double q3 = qt_4x1[3]/temp;
        DC_3x3[0] = 2.*(q0*q0 + q1*q1) - 1.;
        DC_3x3[1] = 2.*(q1*q2 - q0*q3);
        DC_3x3[2] = 2.*(q1*q3 + q0*q2);
        DC_3x3[3] = 2.*(q1*q2 + q0*q3);
        DC_3x3[4] = 2.*(q0*q0 + q2*q2) - 1.;
        DC_3x3[5] = 2.*(q2*q3 - q0*q1);
        DC_3x3[6] = 2.*(q1*q3 - q0*q2);
        DC_3x3[7] = 2.*(q2*q3 + q0*q1);
        DC_3x3[8] = 2.*(q0*q0 + q3*q3) - 1.;
        return 0;
    }
    int QT2YRP(const double qt_4x1[], double &yaw, double &rol, double &pit)
    {
        double dc[9];
        QT2DC(qt_4x1, dc);
        pit = atan2(-dc[2*3+0], dc[2*3+2]);
        yaw = atan2(dc[1*3+0]*cos(pit) + dc[1*3+2]*sin(pit), dc[0*3+0]*cos(pit) + dc[0*3+2]*sin(pit));
        rol = atan2(dc[2*3+1], dc[2*3+2]*cos(pit) - dc[2*3+0]*sin(pit));
        return 0;
    }

    public:

    HB_inverse();
    vec3 FKCOM_UB(ArmJoints AJ);
    ArmPos FK_UB(ArmJoints AJ);
    FeetPos FK(LegJoints LJ);
    LegJoints IK_pel(vec3 pPel, quat qPel, vec3 pRF, quat qRF, vec3 pLF, quat qLF);
    LegJoints IK_pel_drc(vec3 pPel, quat qPel, vec3 pRF, quat qRF, vec3 pLF, quat qLF);
    LegJoints IK_COM(vec3 pCOM, quat qPel, vec3 pRF, quat qRF, vec3 pLF, quat qLF);
    LegJoints IK_COM_xy(vec3 pCOM_des, double pPelz, quat qPel, vec3 pRF, quat qRF, vec3 pLF, quat qLF);

};


#endif // HBINVERSE

