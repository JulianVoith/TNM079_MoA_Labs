#pragma once

#include "Levelset/LevelSetOperator.h"
#include "Math/Function3D.h"
#include "Util/Stopwatch.h"

/*! \brief A level set operator that does external advection
 *
 * This class implements level set advectionr in an external vector field by the
 * PDE
 *
 *  \f$
 *  \dfrac{\partial \phi}{\partial t} + \mathbf{V}(\mathbf{x})\cdot \nabla \phi
 * = 0 \f$
 */
//! \lab4 Implement advection in external vector field
class OperatorAdvect : public LevelSetOperator {
protected:
    Function3D<glm::vec3>* mVectorField;

public:
    OperatorAdvect(LevelSet* LS, Function3D<glm::vec3>* vf)
        : LevelSetOperator(LS), mVectorField(vf) {}

    virtual float ComputeTimestep() {
        // Compute and return a stable timestep
        // (Hint: Function3D::GetMaxValue())
        float dx = mLS->GetDx();
        float tX = dx / std::abs(mVectorField->GetMaxValue().x);
        float tY = dx / std::abs(mVectorField->GetMaxValue().y);
        float tZ = dx / std::abs(mVectorField->GetMaxValue().z);

        return std::min({tX, tY, tZ});
    }

    virtual void Propagate(float time) {
        Stopwatch stop = Stopwatch();
        stop.start();

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
        std::cout << "Advect timestamp: " << stop.read() << std::endl;
    }

    virtual float Evaluate(size_t i, size_t j, size_t k) {
        // Compute the rate of change (dphi/dt)
        
        float x = (float) i;
        float y = (float) j;
        float z = (float) k;
        glm::vec3 gradient;


        mLS->TransformGridToWorld(x, y, z);

        glm::vec3 v = mVectorField->GetValue(x, y, z);

        if (v[0] > 0.0f) {
            gradient.x = mLS->DiffXm(i,j,k);
        } else {
            gradient.x = mLS->DiffXp(i, j, k);
        }

        if (v[1] > 0.0f) {
            gradient.y = mLS->DiffYm(i, j, k);
        } else {
            gradient.y = mLS->DiffYp(i, j, k);
        }
        
        if (v[2] > 0.0f) {
            gradient.z = mLS->DiffZm(i, j, k);
        } else {
            gradient.z = mLS->DiffZp(i, j, k);
        }

        return glm::dot(-1.f*v, gradient);




        // Remember that the point (i,j,k) is given in grid coordinates, while
        // the velocity field used for advection needs to be sampled in
        // world coordinates (x,y,z). You can use LevelSet::TransformGridToWorld()
        // for this task.



        return 0.f;
    }
};