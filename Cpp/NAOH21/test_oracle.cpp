#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include "NAOKinematics.h"
#include "KMat.hpp"
#include "robotConsts.h"

using namespace std;
using namespace KDeviceLists;

void printMatrix(const NAOKinematics::kmatTable& m) {
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            if (r < 3) {
                cout << setprecision(16) << m(r, c) << " ";
            } else {
                cout << (c == 3 ? "1.0 " : "0.0 ");
            }
        }
    }
    cout << "\n";
}

void printCase(NAOKinematics& nkin, const string& caseName, const vector<float>& joints) {
    nkin.setJoints(joints);
    cout << "CASE: " << caseName << "\n";

    // 7 matrices: 5 chains + bottom camera + top camera
    printMatrix(nkin.getForwardEffector((NAOKinematics::Effectors)CHAIN_HEAD));
    printMatrix(nkin.getForwardEffector((NAOKinematics::Effectors)CHAIN_L_ARM));
    printMatrix(nkin.getForwardEffector((NAOKinematics::Effectors)CHAIN_L_LEG));
    printMatrix(nkin.getForwardEffector((NAOKinematics::Effectors)CHAIN_R_LEG));
    printMatrix(nkin.getForwardEffector((NAOKinematics::Effectors)CHAIN_R_ARM));
    printMatrix(nkin.getForwardEffector(NAOKinematics::EFF_CAMERA_BOT));
    printMatrix(nkin.getForwardEffector(NAOKinematics::EFF_CAMERA_TOP));

    KVecDouble3 com = nkin.calculateCenterOfMass();
    cout << "COM: " << setprecision(16) << com(0, 0) << " " << com(1, 0) << " " << com(2, 0) << "\n";
}

int main() {
    NAOKinematics nkin;

    // Case 1: ZERO
    vector<float> j_zero(NUMOFJOINTS, 0.0f);
    printCase(nkin, "ZERO", j_zero);

    // Case 2: NOMINAL
    vector<float> j_nom(NUMOFJOINTS, 0.0f);
    j_nom[L_ARM + SHOULDER_PITCH] = M_PI / 2.0;
    j_nom[L_ARM + SHOULDER_ROLL] = M_PI / 4.0;
    j_nom[R_ARM + SHOULDER_PITCH] = M_PI / 2.0;
    j_nom[R_ARM + SHOULDER_ROLL] = -M_PI / 4.0;
    j_nom[L_LEG + KNEE_PITCH] = M_PI / 4.0;
    j_nom[L_LEG + ANKLE_PITCH] = -M_PI / 4.0;
    j_nom[HEAD + YAW] = M_PI / 2.0;
    printCase(nkin, "NOMINAL", j_nom);

    // Case 3: ARBITRARY
    vector<float> j_arb(NUMOFJOINTS, 0.0f);
    j_arb[HEAD + YAW] = 0.3;
    j_arb[HEAD + PITCH] = -0.2;
    j_arb[L_ARM + SHOULDER_PITCH] = 0.5;
    j_arb[L_ARM + SHOULDER_ROLL] = 0.4;
    j_arb[L_ARM + ELBOW_YAW] = -0.6;
    j_arb[L_ARM + ELBOW_ROLL] = -0.7;
    j_arb[R_ARM + SHOULDER_PITCH] = -0.5;
    j_arb[R_ARM + SHOULDER_ROLL] = -0.4;
    j_arb[R_ARM + ELBOW_YAW] = 0.6;
    j_arb[R_ARM + ELBOW_ROLL] = 0.7;
    j_arb[L_LEG + HIP_YAW_PITCH] = -0.2;
    j_arb[L_LEG + HIP_ROLL] = 0.1;
    j_arb[L_LEG + HIP_PITCH] = -0.3;
    j_arb[L_LEG + KNEE_PITCH] = 0.6;
    j_arb[L_LEG + ANKLE_PITCH] = -0.3;
    j_arb[L_LEG + ANKLE_ROLL] = 0.1;
    j_arb[R_LEG + HIP_YAW_PITCH] = -0.2;
    j_arb[R_LEG + HIP_ROLL] = -0.1;
    j_arb[R_LEG + HIP_PITCH] = -0.3;
    j_arb[R_LEG + KNEE_PITCH] = 0.6;
    j_arb[R_LEG + ANKLE_PITCH] = -0.3;
    j_arb[R_LEG + ANKLE_ROLL] = -0.1;
    printCase(nkin, "ARBITRARY", j_arb);

    // FROM_TO_LLEG_RLEG under ARBITRARY pose
    cout << "FROM_TO_LLEG_RLEG:\n";
    NAOKinematics::kmatTable from_to = nkin.getForwardFromTo((NAOKinematics::Effectors)CHAIN_L_LEG, (NAOKinematics::Effectors)CHAIN_R_LEG);
    printMatrix(from_to);

    return 0;
}
