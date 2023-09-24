/*************************************************************************************************
 *
 * Modeling and animation (TNM079) 2007
 * Code base for lab assignments. Copyright:
 *   Gunnar Johansson (gunnar.johansson@itn.liu.se)
 *   Ken Museth (ken.museth@itn.liu.se)
 *   Michael Bang Nielsen (bang@daimi.au.dk)
 *   Ola Nilsson (ola.nilsson@itn.liu.se)
 *   Andreas Sderstrm (andreas.soderstrom@itn.liu.se)
 *
 *************************************************************************************************/
#pragma once

#include "Levelset/LevelSetOperator.h"

/*! \brief A level set operator that does mean curvature flow.
 *
 * This class implements level set propagation in the normal direction
 * as defined by the mean curvature flow \f$\kappa\f$ in the following PDE
 *
 *  \f[
 *  \dfrac{\partial \phi}{\partial t} + \alpha \kappa|\nabla \phi| = 0
 *  \f]
 */
//! \lab4 Implement mean curvature flow
class OperatorMeanCurvatureFlow : public LevelSetOperator {
protected:
    //! Scaling parameter, affects time step constraint
    float mAlpha;

public:
    OperatorMeanCurvatureFlow(LevelSet* LS, float alpha = 0.9f)
        : LevelSetOperator(LS), mAlpha(alpha) {}

    virtual float ComputeTimestep() {
        // Compute and return a stable timestep
        return ((std::pow(mLS->GetDx(),2)) / (6*mAlpha));
    }

    virtual void Propagate(float time) {
        // Determine timestep for stability
        float dt = ComputeTimestep();

        // Propagate level set with stable timestep dt
        // until requested time is reached
        for (float elapsed = 0.f; elapsed < time;) {
            if (dt > time - elapsed) {
                dt = time - elapsed;
            }
            elapsed += dt;

            IntegrateEuler(dt);
            // IntegrateRungeKutta(dt);
        }
    }

    virtual float Evaluate(size_t i, size_t j, size_t k) {
        // Compute the rate of change (dphi/dt)

        float kX =
            ((std::pow(mLS->DiffXpm(i, j, k), 2) * (mLS->Diff2Ypm(i, j, k) + mLS->Diff2Zpm(i, j, k))) - (2 * mLS->DiffYpm(i, j, k) * mLS->DiffZpm(i, j, k) * mLS->Diff2YZpm(i, j, k))) /
            (2 * std::pow((std::pow(mLS->DiffXpm(i, j, k), 2) + std::pow(mLS->DiffYpm(i, j, k), 2) + std::pow(mLS->DiffZpm(i, j, k), 2)), (3 / 2)));
        float kY =
            ((std::pow(mLS->DiffYpm(i, j, k), 2) * (mLS->Diff2Xpm(i, j, k) + mLS->Diff2Zpm(i, j, k))) - (2 * mLS->DiffXpm(i, j, k) * mLS->DiffZpm(i, j, k) * mLS->Diff2ZXpm(i, j, k))) /
            (2 * std::pow((std::pow(mLS->DiffXpm(i, j, k), 2) + std::pow(mLS->DiffYpm(i, j, k), 2) + std::pow(mLS->DiffZpm(i, j, k), 2)), (3 / 2)));
        float kZ =
            ((std::pow(mLS->DiffZpm(i, j, k), 2) * (mLS->Diff2Xpm(i, j, k) + mLS->Diff2Ypm(i, j, k))) - (2 * mLS->DiffXpm(i, j, k) * mLS->DiffYpm(i, j, k) * mLS->Diff2XYpm(i, j, k))) /
            (2 * std::pow((std::pow(mLS->DiffXpm(i, j, k), 2) + std::pow(mLS->DiffYpm(i, j, k), 2) + std::pow(mLS->DiffZpm(i, j, k), 2)), (3 / 2)));

        float curv = kX + kY + kZ;

        glm::vec3 gradient = {mLS->DiffXpm(i, j, k), mLS->DiffYpm(i, j, k), mLS->DiffZpm(i, j, k)};

        float normGrad = gradient.length();

        return mAlpha*curv*normGrad;
    }
};
