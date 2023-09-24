#ifndef _strange_dubdivmesh_
#define _strange_dubdivmesh_

#include "AdaptiveLoopSubdivisionMesh.h"
#include <gtx/vector_angle.hpp>
#include <math.h>

class StrangeSubdivisionMesh : public AdaptiveLoopSubdivisionMesh {
public:
    virtual void Subdivide() {
        AdaptiveLoopSubdivisionMesh::Subdivide();
    }

protected:
    bool Subdividable(size_t fi) {
        // Every 4th face is not subdividable - kinda strange!
        // Do something more interesting...
                
        HalfEdgeMesh::Face currentFace = f(fi);
        //HalfEdgeMesh::Face neighbourFace = f(e(e(currentFace.edge).pair).face);

        std::vector<size_t> oneRing = HalfEdgeMesh::FindNeighborFaces(e(f(fi).edge).vert);
        float angle = 0;
        float angleCurr = 0;

        for (auto face : oneRing) {
            
            
            angleCurr = glm::angle(glm::normalize(currentFace.normal), glm::normalize(f(face).normal));

            if (angleCurr > angle) {
                angle = angleCurr;
            }
        }


        if (abs(angle) <= M_PI/3) {

            return true;
        } else {
            return false;
        }

       // return (fi % 4);
    }
};

#endif
