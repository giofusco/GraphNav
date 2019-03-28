#if !defined(MAPMANAGER_HPP_)
#define MAPMANAGER_HPP_

#include "AnnotatedMap.hpp"
#include "../Utils/ParseUtils.hpp"
#include <opencv2/core/core.hpp>

#include <map>
#include <iostream>
#include <stdlib.h>


namespace maps{
    
    const std::string _PARSER_LOCATION_NAME_TAG = "building_name";
    const std::string _PARSER_FLOOR_TAG = "floor";
    const std::string _PARSER_ID_TAG = "id";
    const std::string _PARSER_FEATURES_FILE_TAG = "features_file";
    const std::string _PARSER_WALLS_TAG = "walls";
    const std::string _PARSER_WALKABLE_TAG = "walkable";
    const std::string _PARSER_ROIS_TAG = "rois";
    const std::string _PARSER_ROIS_DICTIONARY_TAG = "rois_dictionary";
    const std::string _PARSER_SCALE_TAG = "scale";
    const std::string _PARSER_MAN_DELIMITER = ":";
    const int         _PARSER_BLOCK_LEN = 7;

using FloorNumber = int;

class MapManager{

    public:
      
        int currentFloor;

        MapManager(){_TAG = "MapManager";}
        //MapManager(const MapManager& mapManager) = default;
    
        void init(std::string imapFolder, int icurrentFloor){
            _mapFolder = imapFolder;
            currentFloor = icurrentFloor;
            _mapFile = _mapFolder + "/info.yml";
            _loadMaps();
        }

        inline cv::Point2i uv2pixels(cv::Point2f pt)      { return _maps.at(currentFloor).uv2pixels(pt); }
        inline cv::Point2i uv2pixels(cv::Vec2f pt)        { return _maps.at(currentFloor).uv2pixels(cv::Point2f(pt[0], pt[1])); }
        inline cv::Vec2i   uv2pixelsVec(cv::Vec2f pt)     { cv::Point2i tmp = _maps.at(currentFloor).uv2pixels(cv::Point2f(pt[0], pt[1]));
                                                            return cv::Vec2i(tmp.x, tmp.y);}
        inline cv::Point2d pixels2uv(cv::Point2i pt) { return _maps.at(currentFloor).pixels2uv(pt); };
    
        inline const cv::Mat getWallsImage()            { return _maps.at(currentFloor).getWallsImage(); }
        inline const cv::Mat getWallsImageRGB()            { return _maps.at(currentFloor).getWallsImageRGB(); }
    
        inline cv::Size mapSizeMeters(){ return _maps.at(currentFloor).getMapSizeMeters(); }
        inline cv::Size getMapSizePixels()      { return _maps.at(currentFloor).getMapSizePixels(); }
        inline bool isWalkable(cv::Point2i pt) { return _maps.at(currentFloor).isWalkable(pt); }
        inline bool isWallAt(cv::Point2i pt)   { return _maps.at(currentFloor).isWallAt(pt); }
        inline std::multimap<FeatureType, MapFeature> getLandmarksList() const { return _maps.at(currentFloor).getLandmarksList(); }
        inline void showFeatures() { _maps.at(currentFloor).showFeatures();}
    
        inline std::string getClosestPOI(cv::Point2i pt) { return _maps.at(currentFloor).getClosestPOI(pt); }
        inline int getRoiAt(cv::Point2i pt) { return _maps.at(currentFloor).getRoiAt(pt); }
        inline std::string getRoiLabel(int idx) { return _maps.at(currentFloor).getRoiLabel(idx); }
        inline std::string getRoiLabelAt(cv::Point2i pt) {
            std::string label = "" ;
            int id = _maps.at(currentFloor).getRoiAt(pt);
            if (id >0)
                label = getRoiLabel(id);
            return label;
        }
    
        inline double getScale() { return _maps.at(currentFloor).getScale();}
    
        bool isPathCrossingWalls(cv::Point2i startPt, cv::Point2i endPt){
            // iterate over pixels from startPt to endPT to check if walls are in the way of the path
            int dr = abs(startPt.x - endPt.x);
            int dc = abs(startPt.y - endPt.y);
            
            int span = std::max(dr, dc);  // if more rows than columns then loop over rows; else loop over columns
            float multiplier;
            if (span == 0)  //special case: (r1,c1) equals (r2,c2)
                multiplier = 0.;
            else
                multiplier = 1. / span;
            
            for ( int k=0; k <= span; k++){ // k goes from 0 through span; e.g., a span of 2 implies there are 2+1=3 pixels to reach in loop
                float frac = k * multiplier;
                int r = std::trunc(startPt.x + frac * (endPt.x - startPt.x));
                int c = std::trunc(startPt.y + frac * (endPt.y - startPt.y));
                if (isWallAt(cv::Point(r, c))>0)
                    return true;
            }
            return false;
        }


    private:
        std::string _mapFolder;
        std::string _mapFile;
        std::string _currentLocationName;
        std::map<FloorNumber, AnnotatedMap> _maps;
        std::string _TAG;
    
        void _parseFloorBlock(std::ifstream& inFile, std::map<std::string, std::string>& mapDetails){
            int lineCnt = 0;
            std::string strLine;
        
            while (lineCnt < _PARSER_BLOCK_LEN){
                getline(inFile, strLine);
                mapDetails.insert(parseutils::splitKeyValue(strLine, _PARSER_DELIMITER));
                lineCnt++;
            }
        }
    
        void _loadMaps() {
            
            std::ifstream inFile(_mapFile, std::ifstream::in);
            std::string strLine;
            while (inFile.good()){
                getline(inFile, strLine);
                if (strLine.size() > 0){
                    parseutils::KeyValuePair keyValPair = parseutils::splitKeyValue(strLine, _PARSER_DELIMITER);
                    
                    // the tag specifies the name of the building
                    if (keyValPair.first == _PARSER_LOCATION_NAME_TAG)
                        _currentLocationName = keyValPair.second;
                    // parse floor block
                    else if (keyValPair.first == _PARSER_FLOOR_TAG){
                        
                        std::map<std::string, std::string> mapDetails;
                        _parseFloorBlock(inFile, mapDetails);
                        
                        // init Annotated Map Object
                        _maps.insert(std::make_pair(std::stoi(mapDetails[_PARSER_ID_TAG]), AnnotatedMap(mapDetails[_PARSER_WALLS_TAG], mapDetails[_PARSER_WALKABLE_TAG], mapDetails[_PARSER_FEATURES_FILE_TAG], mapDetails[_PARSER_ROIS_TAG], mapDetails[_PARSER_ROIS_DICTIONARY_TAG], std::stof(mapDetails[_PARSER_SCALE_TAG]), _mapFolder)) );
                    }
                }
            }
            inFile.close();
        }
};

} //::map

#endif // MAPMANAGER_HPP_
