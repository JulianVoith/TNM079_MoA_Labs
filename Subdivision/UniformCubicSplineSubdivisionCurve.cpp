#include <Subdivision/UniformCubicSplineSubdivisionCurve.h>
#include <glm.hpp>
#include <gtc/type_ptr.hpp>

UniformCubicSplineSubdivisionCurve::UniformCubicSplineSubdivisionCurve(
    const std::vector<glm::vec3>& joints, glm::vec3 lineColor, float lineWidth)
    : mCoefficients(joints), mControlPolygon(joints) {
    this->mLineColor = lineColor;
    this->mLineWidth = lineWidth;
}

void UniformCubicSplineSubdivisionCurve::Subdivide() {
    // Allocate space for new coefficients
    std::vector<glm::vec3> newc;

    assert(mCoefficients.size() > 4 && "Need at least 5 points to subdivide");

    for (int i = 0; i < mCoefficients.size(); i++) {
        if (i == 0) {
            newc.push_back(mCoefficients.at(i));
            newc.push_back((4.0f * mCoefficients.at(i) + 4.0f * mCoefficients.at(i + 1)) / 8.0f);
        } else if (i == mCoefficients.size() - 1) {
            newc.push_back(mCoefficients.at(i));
        } else {
            newc.push_back(
                (mCoefficients.at(i - 1) + 6.0f * mCoefficients.at(i) + mCoefficients.at(i + 1)) /
                8.0f);
            newc.push_back((4.0f * mCoefficients.at(i) + 4.0f * mCoefficients.at(i + 1)) / 8.0f);
        }
    }

    // Implement the subdivision scheme for a natural cubic spline here

    // If 'mCoefficients' had size N, how large should 'newc' be? Perform a check
    // here!
    if (newc.size() == mCoefficients.size() * 2 - 1) {
        mCoefficients = newc;

    } else {
        assert(true && "Incorrect number of new coefficients!");
    }
}

void UniformCubicSplineSubdivisionCurve::Render() {
    // Apply transform
    glPushMatrix();  // Push modelview matrix onto stack

    // Convert transform-matrix to format matching GL matrix format
    // Load transform into modelview matrix
    glMultMatrixf(glm::value_ptr(mTransform));

    mControlPolygon.Render();

    // save line point and color states
    glPushAttrib(GL_POINT_BIT | GL_LINE_BIT | GL_CURRENT_BIT);

    // draw segments
    glLineWidth(mLineWidth);
    glColor3fv(glm::value_ptr(mLineColor));
    glBegin(GL_LINE_STRIP);
    // just draw the spline as a series of connected linear segments
    for (size_t i = 0; i < mCoefficients.size(); i++) {
        glVertex3fv(glm::value_ptr(mCoefficients.at(i)));
    }
    glEnd();

    // restore attribs
    glPopAttrib();

    glPopMatrix();

    GLObject::Render();
}
