/*************************************************************************************************
 *
 * Modeling and animation (TNM079) 2007
 * Code base for lab assignments. Copyright:
 *   Gunnar Johansson (gunnar.johansson@itn.liu.se)
 *   Ken Museth (ken.museth@itn.liu.se)
 *   Michael Bang Nielsen (bang@daimi.au.dk)
 *   Ola Nilsson (ola.nilsson@itn.liu.se)
 *   Andreas Söderström (andreas.soderstrom@itn.liu.se)
 *
 *************************************************************************************************/

#include "Util/IsoContourColorMap.h"

ColorMapFactory::FactoryRegistration IsoContourColorMap::mFactoryRegistration(
    "Iso contour", new IsoContourColorMap());

IsoContourColorMap::IsoContourColorMap() {
    mColors.push_back(glm::vec3(0.f, 1.f, 0.f));
    mColors.push_back(glm::vec3(1.f, 0.f, 0.f));
}

glm::vec3 IsoContourColorMap::Map(float val, float low, float high) const {
    // Take absolute value
    if (val < 0.f) {
        val = -val;
    }
    // Compute fraction to do wrap-around
    float fraction = val * 10.f - (size_t)(val * 10.f);

    // Do color mapping
    return ColorMap::Map(fraction, 0.f, 1.f);
}
