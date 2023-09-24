/*************************************************************************************************
 *
 * Modeling and animation (TNM079) 2007
 * Code base for lab assignments. Copyright:
 * Acknowledgements for original code base:
 * - Gunnar Johansson
 * - Ken Museth
 * - Michael Bang Nielsen
 * - Ola Nilsson
 * - Andreas Soderstrom
 *
 * Code updated in the period 2017-2018 by Jochen Jankowai
 *
 *************************************************************************************************/
#include <Geometry/LineStrip.h>
#include <glm.hpp>
#include <gtc/type_ptr.hpp>

LineStrip::LineStrip(const std::vector<glm::vec3>& joints) : mJoints(joints) {
    mJointColor = glm::vec3(1.f, 0.f, 0.f);
    mLineColor = glm::vec3(0.f, 0.f, 1.f);
    mLineWidth = 3.f;
    mJointSize = 5.f;
}

void LineStrip::Render() {
    // save line point and color states
    glPushAttrib(GL_POINT_BIT | GL_LINE_BIT | GL_CURRENT_BIT);

    // draw joints
    glPointSize(mJointSize);
    glColor3fv(glm::value_ptr(mJointColor));
    glBegin(GL_POINTS);
    for (auto it = mJoints.begin(); it != mJoints.end(); it++) {
        glVertex3fv(glm::value_ptr(*it));
    }
    glEnd();

    // draw segments
    glLineWidth(mLineWidth);
    glColor3fv(glm::value_ptr(mLineColor));
    glBegin(GL_LINE_STRIP);
    for (auto it = mJoints.begin(); it != mJoints.end(); it++) {
        glVertex3fv(glm::value_ptr(*it));
    }
    glEnd();

    // restore attribs
    glPopAttrib();

    GLObject::Render();
}
