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
#include "Cube.h"

Cube::Cube() {
    glm::vec3 lowP(-1.1f, -1.1f, -1.1f);
    glm::vec3 highP(1.1f, 1.1f, 1.1f);
    SetBoundingBox(Bbox(lowP, highP));

    glm::mat4 matrix{0.f};

    // Plane 1
    matrix[3][1] = 0.5f;
    Quadric *q = new Quadric(matrix);
    q->Translate(0.f, 0.5f, 0.f);
    q->SetBoundingBox(Bbox(lowP, highP));
    mPlanes.push_back(q);

    // Plane 2
    matrix[3][1] = -0.5f;
    q = new Quadric(matrix);
    q->Translate(0.f, -0.5f, 0.f);
    q->SetBoundingBox(Bbox(lowP, highP));
    mPlanes.push_back(q);

    // Plane 3
    matrix[3][0] = 0.5f;
    matrix[3][1] = 0.f;
    q = new Quadric(matrix);
    q->Translate(0.5f, 0.f, 0.f);
    q->SetBoundingBox(Bbox(lowP, highP));
    mPlanes.push_back(q);

    // Plane 4
    matrix[3][0] = -0.5f;
    q = new Quadric(matrix);
    q->Translate(-0.5f, 0.f, 0.f);
    q->SetBoundingBox(Bbox(lowP, highP));
    mPlanes.push_back(q);

    // Plane 5
    matrix[3][0] = 0.f;
    matrix[3][2] = 0.5f;
    q = new Quadric(matrix);
    q->Translate(0.f, 0.f, 0.5f);
    q->SetBoundingBox(Bbox(lowP, highP));
    mPlanes.push_back(q);

    // Plane 6
    matrix[3][2] = -0.5f;
    q = new Quadric(matrix);
    q->Translate(0.f, 0.f, -0.5f);
    q->SetBoundingBox(Bbox(lowP, highP));
    mPlanes.push_back(q);
}

Cube::~Cube() {
    for (size_t i = 0; i < mPlanes.size(); i++) {
        delete mPlanes[i];
    }
}

float Cube::GetValue(float x, float y, float z) const {
    TransformW2O(x, y, z);

    float value = mPlanes[0]->GetValue(x, y, z);
    for (size_t i = 1; i < mPlanes.size(); i++) {
        value = std::max(value, mPlanes[i]->GetValue(x, y, z));
    }

    return value;
}
