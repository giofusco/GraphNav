#if !defined(MAPFEATURE_HPP_)
#define MAPFEATURE_HPP_

#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace maps{

    enum FeatureType {ARUCO, EXIT_SIGN};
    
    
struct MapFeature{

    std::string description;
    FeatureType type;
    cv::Point2f position;
    cv::Vec2d normal;
    
    cv::Vec2d getPositionVector() { return cv::Vec2d(position.x, position.y);}

    //MapFeature();
    
};

} // ::map

#endif // MAPFEATURE_HPP_
