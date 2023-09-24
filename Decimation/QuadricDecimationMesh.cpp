#include "QuadricDecimationMesh.h"
#include "gtc/type_ptr.hpp"

const QuadricDecimationMesh::VisualizationMode QuadricDecimationMesh::QuadricIsoSurfaces =
    NewVisualizationMode("Quadric Iso Surfaces");

void QuadricDecimationMesh::Initialize() {
    // Allocate memory for the quadric array
    size_t numVerts = mVerts.size();
    mQuadrics.reserve(numVerts);
    std::streamsize width = std::cerr.precision();  // store stream precision
    for (size_t i = 0; i < numVerts; i++) {

        // Compute quadric for vertex i here
        mQuadrics.push_back(createQuadricForVert(i));

        // Calculate initial error, should be numerically close to 0

        glm::vec3 v0 = mVerts[i].pos;
        glm::vec4 v(v0[0], v0[1], v0[2], 1);
        glm::mat4 m = mQuadrics.back();

        // TODO CHECK
        float error = glm::dot(v, (m * v));
        // std::cerr << std::scientific << std::setprecision(2) << error << " ";
    }
    std::cerr << std::setprecision(width) << std::fixed;  // reset stream precision

    // Run the initialize for the parent class to initialize the edge collapses
    DecimationMesh::Initialize();
}

/*! \lab2 Implement the computeCollapse here */
/*!
 * \param[in,out] collapse The edge collapse object to (re-)compute,
 * DecimationMesh::EdgeCollapse
 */
void QuadricDecimationMesh::computeCollapse(EdgeCollapse* collapse) {
    // Compute collapse->position and collapse->cost here
    // based on the quadrics at the edge endpoints

    size_t v1point = e(collapse->halfEdge).vert;
    glm::vec4 v1 = glm::vec4(v(v1point).pos, 1.0f);

    size_t v2point = e(e(collapse->halfEdge).pair).vert; 
    glm::vec4 v2 = glm::vec4(v(v2point).pos, 1.0f);

    glm::vec4 v3((v(v1point).pos + v(v2point).pos) / 2.0f, 1.0f);

    glm::mat4 q1 = mQuadrics.at(v1point);
    glm::mat4 q2 = mQuadrics.at(v2point);

    glm::mat4 qTot = q1 + q2;
    glm::mat4 qTotOpti = q1 + q2;

    qTotOpti[3][0] = 0;
    qTotOpti[3][1] = 0;
    qTotOpti[3][2] = 0;
    qTotOpti[3][3] = 1;

    //Task for a 4
    HalfEdgeMesh::Face face = f(e(collapse->halfEdge).face);
    float weight = 1.0f;
    if (face.normal.y >= 0.3) {
        weight = face.normal.y * 20;
    }
    
    
    if (glm::determinant(qTotOpti) != 0) {
        glm::vec4 optimalPos = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f) *
                               glm::inverse(qTotOpti);  //why vector times mat and not vice versa?

        float error = glm::dot((optimalPos * qTot), optimalPos);
       
        collapse->cost = error * weight;

        collapse->position = glm::vec3{optimalPos[0], optimalPos[1], optimalPos[2]};
    }
    else { 

        float error1 = glm::dot((v1 * qTot), v1);
        float error2 = glm::dot((v2 * qTot), v2);
        float error3 = glm::dot((v3 * qTot), v3);

        float min_error = std::min({error1, error2, error3});

        collapse->cost = min_error * weight;

        if (min_error == error1) {
            collapse->position = v(v1point).pos;
        } else if (min_error == error2) {
            collapse->position = v(v2point).pos;
        } else {
            collapse->position = (v(v1point).pos + v(v2point).pos) / 2.0f;
        }
    }

    //std::cout << v(v1point).pos.y << std::endl;
   // std::cerr << "computeCollapse in QuadricDecimationMesh not implemented.\n";
}

/*! After each edge collapse the vertex properties need to be updated */
void QuadricDecimationMesh::updateVertexProperties(size_t ind) {
    DecimationMesh::updateVertexProperties(ind);
    mQuadrics[ind] = createQuadricForVert(ind);
}

/*!
 * \param[in] indx vertex index, points into HalfEdgeMesh::mVerts
 */
glm::mat4 QuadricDecimationMesh::createQuadricForVert(size_t indx) const {
    glm::mat4 Q({0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f},
                {0.0f, 0.0f, 0.0f, 0.0f});

    // The quadric for a vertex is the sum of all the quadrics for the adjacent
    // faces Tip: Matrix4x4 has an operator +=
    
    std::vector<size_t> oneRing = HalfEdgeMesh::FindNeighborFaces(indx);
    

    for (auto face : oneRing) {
        Q += createQuadricForFace(face);   
    }

    return Q;
}

/*!
 * \param[in] indx face index, points into HalfEdgeMesh::mFaces
 */
glm::mat4 QuadricDecimationMesh::createQuadricForFace(size_t indx) const {

    // Calculate the quadric (outer product of plane parameters) for a face
    // here using the formula from Garland and Heckbert
    
    glm::vec3 faceNormal = f(indx).normal;
    glm::vec3 vZero = v(e(f(indx).edge).vert).pos;

    float d = -glm::dot(vZero, faceNormal); 
    glm::vec4 p(faceNormal.x, faceNormal.y, faceNormal.z, d);

    glm::mat4 quadric_matrix({0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f},
                             {0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f});

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            quadric_matrix[i][j] = p[i] * p[j];

        }
    }

    return quadric_matrix;
}

void QuadricDecimationMesh::Render() {
    DecimationMesh::Render();

    glEnable(GL_LIGHTING);
    glMatrixMode(GL_MODELVIEW);

    if (mVisualizationMode == QuadricIsoSurfaces) {
        int i = 0;
        // Apply transform
        glPushMatrix();  // Push modelview matrix onto stack
		glMultMatrixf(glm::value_ptr(mTransform));
        
        GLUquadric* GLquad = gluNewQuadric();
        glColor3f(0, 1, 0);

        for (auto quad : mQuadrics){
			glm::mat4 R({ 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f },
				{ 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f });
			bool isFactorized = CholeskyFactorization(quad, R);
            
			if (isFactorized && !isVertexCollapsed(i)){
                glPushMatrix();
				R = glm::inverse(R);
				
				glMultMatrixf(glm::value_ptr(R));

				gluSphere(GLquad, 3, 10, 10);

				glPopMatrix();
			}
         ++i;
       
		}

        // Restore modelview matrix
        glPopMatrix();
    }
}
