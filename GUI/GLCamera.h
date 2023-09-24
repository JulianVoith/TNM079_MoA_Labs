
#ifdef __WXMAC__
#include "GLUT/glut.h"
#else
#include <GL/glut.h>
#endif

#include <Util/trackball.h>
#include <glm.hpp>
/////////////////////////////////
// Note: All angles in degrees  //
/////////////////////////////////

class GLCamera {
private:
    glm::vec3 mCameraFwd;
    glm::vec3 mCameraPos;
    glm::vec3 mCameraUp;
    float curquat[4];  // a representation of the rotation quaternion
    bool useCameraRotation;

public:
    GLCamera(glm::vec3 v = glm::vec3(0.f, 0.f, 0.f)) {
        mCameraPos = v;
        mCameraUp = glm::vec3(0.f, 1.f, 0.f);
        mCameraFwd = glm::vec3(0.f, 0.f, -1.f);
        mCameraFwd = glm::normalize(mCameraFwd);
        trackball(curquat, 0, 0, 0, 0);
        useCameraRotation = false;
    }
    void SetCameraRotation(bool r) { useCameraRotation = r; }

    void RotateX(float r) {
        if (useCameraRotation) {
            r = r * (1.f / 180.f);
            mCameraUp = cos(r) * mCameraUp + sin(r) * mCameraFwd;
            mCameraFwd = cos(r) * mCameraFwd - sin(r) * mCameraUp;
        }
    }
    void RotateY(float r) {
        if (useCameraRotation) {
            r = r * (1.f / 180.f);
            mCameraFwd = cos(r) * mCameraFwd + sin(r) * GetRightVector();
        }
    }
    void RotateZ(float r) {
        if (useCameraRotation) {
            r = r * (1.f / 180.f);
            mCameraUp = cos(r) * mCameraUp + sin(r) * GetRightVector();
        }
    }

    void QuatRotate(float x1, float y1, float x2, float y2) {
        float lastquat[4];
        trackball(lastquat, x1, y1, x2, y2);  // calculates a reasonable quaternion
                                              // using the start and end points
        add_quats(lastquat, curquat,
                  curquat);  // Quats can be accumulated for a smooth rotation effect
    }

    glm::vec3 GetPosition() const { return mCameraPos; }
    glm::vec3 GetLookAtVector() const { return mCameraFwd; }
    glm::vec3 GetRightVector() const { return glm::cross(GetLookAtVector(), mCameraUp); }
    glm::vec3 GetUpVector() const { return mCameraUp; }
    glm::vec3 GetLookAtPoint() const { return glm::vec3(mCameraPos + mCameraFwd); }

    void LookAtOrigo() { mCameraFwd = -mCameraPos; }

    void Reset() { *this = GLCamera(); }

    void Move(const glm::vec3& m) { mCameraPos += m; }
    void MoveForward(float r) { Move(GetLookAtVector() * r); }
    void MoveUpward(float r) { Move(GetUpVector() * r); }
    void MoveRight(float r) { Move(GetRightVector() * r); }

    void Render() {
        glm::vec3 lookAtPos = mCameraPos + mCameraFwd;
        gluLookAt(mCameraPos[0], mCameraPos[1], mCameraPos[2], lookAtPos[0], lookAtPos[1],
                  lookAtPos[2], mCameraUp[0], mCameraUp[1], mCameraUp[2]);
        GLfloat m[4][4];
        build_rotmatrix(m, curquat);  // using trackball to compute the equivalent rotation matrix
        glMultMatrixf(&m[0][0]);
    }

    glm::vec3 Spherical2Cartesian(const glm::vec3& spherical) const {
        const float r = spherical[0];
        const float phi = spherical[1];
        const float theta = spherical[2];

        const float x = r * sin(theta) * sin(phi);
        const float y = r * cos(theta);
        const float z = r * sin(theta) * cos(phi);

        return glm::vec3(x, y, z);
    }

    glm::vec3 Cartesian2Spherical(const glm::vec3& cartesian) const {
        const float x = cartesian[0];
        const float y = cartesian[1];
        const float z = cartesian[2];

        const float r = glm::length(cartesian);
        const float theta = acos(y / r);
        const float phi = atan2(x, z);

        return glm::vec3(r, phi, theta);
    }
};
