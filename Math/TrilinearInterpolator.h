#pragma once

#include <Math/Volume.h>

class TrilinearInterpolator {
public:
    TrilinearInterpolator();
    ~TrilinearInterpolator();

    template <typename T>
    T Interpolate(float x, float y, float z, const Volume<T>& grid);
};
