
#include "ColorMapFactory.h"

ColorMap* ColorMapFactory::New(const std::string& name) { return GetMaps()[name]; }

std::list<std::string> ColorMapFactory::GetColorMaps() {
    std::list<std::string> maps;
    std::map<std::string, ColorMap*>& colormaps = GetMaps();
    for (auto const& map : colormaps) {
        maps.push_back(map.first); // the name
    }

    return maps;
}
