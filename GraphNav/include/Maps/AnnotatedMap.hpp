#if !defined(ANNOTATEDMAP_HPP_)
#define ANNOTATEDMAP_HPP_

#include <opencv2/opencv.hpp>

#include "MapFeature.hpp"
#include "../Utils/ParseUtils.hpp" 

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <map>

namespace maps{
    
    const std::string _PARSER_FEATURE_TAG = "map_feature";
    const std::string _PARSER_TYPE_TAG = "id";
    const std::string _PARSER_POSITION_TAG = "position";
    const std::string _PARSER_NORMAL_TAG = "normal";
    const std::string _PARSER_DESCRIPTION_TAG = "name";
    const std::string _PARSER_DELIMITER = ":";
    const int         _PARSER_FLOOR_BLOCK_LEN = 5;

class AnnotatedMap{
    
    public:
    
    AnnotatedMap(std::string wallsImageFile, std::string walkableImageFile, std::string mapLandmarksFile, std::string roisImageFile, std::string roisDictionaryFile,
                 float scale, std::string mapFolder){
            _landmarksFile = mapFolder + '/' + mapLandmarksFile;
            _wallsImageFile = mapFolder + '/' + wallsImageFile;
            _walkableImageFile = mapFolder + '/' + walkableImageFile;
            _roisImageFile = mapFolder + '/' + roisImageFile;
            _roisDictionaryFile = mapFolder + '/' + roisDictionaryFile;
            _scale = scale;

            _loadImageData();
            _loadFeatures();
            _loadRoisDictionary();
        }
    
        AnnotatedMap(const AnnotatedMap& annotatedMap) = default;

        inline cv::Mat getWalkableMask() { return _walkMask; }
        inline cv::Mat getWallsImage()   { return _wallsImage; }

        // Convert u,v position in map to a pixel location in the map image
        cv::Point2i uv2pixels(cv::Point2d pt){ //assumption: All points in the system are stored row majow, i.e. x denotes rows
            cv::Point2i px;
            px.y = _scale * pt.y;
            px.x = _size.height - _scale * pt.x;
            return px;
        }
    
        inline cv::Size getMapSizePixels()      { return _size; }
        inline cv::Size getMapSizeMeters()      { return cv::Size(_size.width/_scale, _size.height/_scale); }
        inline bool isWalkable(cv::Point2i pt)  { if (pt.x < 0 || pt.x >= _size.height || pt.y < 0 || pt.y >= _size.width) return false;
                                                    bool out; _walkMask.at<unsigned char>(pt.x , pt.y) > 0 ? out = true : out = false; return out; }
        inline bool isWallAt(cv::Point2i pt)    { if (pt.x < 0 || pt.x >= _size.height || pt.y < 0 || pt.y >= _size.width) return true;
//                                                  cv::Scalar val = _wallsImage.at<uchar>(pt);
//                                                  cv::Scalar scalar = _wallsImage.at<cv::Scalar_<uchar>>(pt.x, pt.y);
                                                  bool out; _wallsImage.at<uchar>(pt.x, pt.y) > 0 ? out = true : out = false; return out; }
    
        inline std::multimap<FeatureType, MapFeature> getLandmarksList() const { return _landmarks; }
    
        inline std::string getRoiLabel(int idx) {
                auto it = _roisDictionary.find(idx);
                if (it != _roisDictionary.end())
                    return it->second;
                else return "";
        }
    
        void showFeatures(){
            cv::Mat map;
            _wallsImage.copyTo(map);
            cv::cvtColor(map, map, cv::COLOR_GRAY2RGB);
            auto lmarks = _landmarks.find(maps::FeatureType::EXIT_SIGN);
            
            for(auto mf = lmarks; mf != _landmarks.end(); mf++){
                cv::Point2d pt = cv::Point2d(mf->second.position.x, mf->second.position.y);
                cv::Point2i px = uv2pixels(pt);
                cv::circle(map, cv::Point(px.y,px.x)  , 3, cv::Scalar(0,150,255));
                cv::circle(map, cv::Point(px.y,px.x)  , 1, cv::Scalar(0,150,255));
            }
            cv::imshow("Features", map);
        }
    
    cv::Point2d pixels2uv(cv::Point2i pt){
        cv::Point2d pt_mt;
        pt_mt.y = (pt.y) / _scale;
        pt_mt.x = (_size.height - pt.x) / _scale;
        return pt_mt;
    }
    
    inline double getScale() { return _scale; }
    
    cv::Mat getWallsImageRGB() { return _wallsImageRGB; }
    
    inline int getRoiAt(cv::Point2i pt) { return (int) _roisImage.at<unsigned char>(pt.x, pt.y); }
    
    std::string getClosestPOI(cv::Point2i pt){
        cv::Point2f pt_mt = pixels2uv(pt);
        cv::Point2f featPt;
        double minDist = 1e6;
        double thrDist = 1.5;
        std::string label = " ";
        for (auto it = _landmarks.begin(); it != _landmarks.end(); ++it){
            if (it->first != EXIT_SIGN){
                double res = cv::norm(pt_mt-it->second.position);
                if (res < minDist && res <= thrDist){
                    minDist = res;
                    label = it->second.description;
                }
            }
        }
        
        if (minDist <= thrDist)
            return "Near " + label;
        else return " ";
    }
    
    private: 
    
        double _scale;
        
        std::string _landmarksFile;
        std::string _wallsImageFile;
        std::string _walkableImageFile;
        std::string _roisImageFile;
        std::string _roisDictionaryFile;
        
        cv::Mat _wallsImage;
        cv::Mat _walkMask;
        cv::Mat _wallsImageRGB;
        cv::Mat _roisImage;
    
        cv::Size _size;
    
        std::map<int, std::string> _roisDictionary;
        
        std::multimap<FeatureType, MapFeature> _landmarks;
    
    

        void _loadImageData(){
            _wallsImage = cv::imread(_wallsImageFile, cv::IMREAD_GRAYSCALE);
            cv::cvtColor(_wallsImage, _wallsImageRGB, cv::COLOR_GRAY2RGB);
            _walkMask   = cv::imread(_walkableImageFile, cv::IMREAD_GRAYSCALE);
            _size    = cv::Size(_wallsImage.cols, _wallsImage.rows);
            _roisImage = cv::imread(_roisImageFile, cv::IMREAD_GRAYSCALE);
            //cv::threshold(_wallsImage, _wallsImage, 0, 255, cv::THRESH_BINARY);
            //cv::threshold(_walkMask, _walkMask, 0, 255, cv::THRESH_BINARY);
            //cv::imshow("WALK", _walkMask);
        }
    
        void _loadRoisDictionary() {
            std::ifstream inFile(_roisDictionaryFile, std::ifstream::in);
            std::string strLine;
            while (inFile.good()){
                getline(inFile, strLine);
                parseutils::IndexValuePair indexValPair = parseutils::splitIndexValue(strLine, ",");
                _roisDictionary.insert(indexValPair);
            }
        }
    
        void _loadFeatures(){ _parseYamlFeaturesFile();}
    
        MapFeature _initMapFeature(std::ifstream& inFile){
            int lineCnt = 0;
            std::string strLine;
            MapFeature mapFeature;
            
            while (lineCnt < _PARSER_FLOOR_BLOCK_LEN){
                getline(inFile, strLine);
                parseutils::KeyValuePair keyValPair = parseutils::splitKeyValue(strLine, _PARSER_DELIMITER);
                
                if (keyValPair.first == _PARSER_TYPE_TAG){
                    std::size_t found = keyValPair.second.find("aruco");
                    if (found!=std::string::npos)
                        mapFeature.type = ARUCO;
                    else if (keyValPair.second == "exit_sign")
                        mapFeature.type = EXIT_SIGN;
                }
                
                else if (keyValPair.first == _PARSER_DESCRIPTION_TAG)
                    mapFeature.description = keyValPair.second;
                
                else if (keyValPair.first == _PARSER_POSITION_TAG){
                    std::vector<double> v = parseutils::parseCSVArray<double>(keyValPair.second);
                    mapFeature.position = cv::Point2f(v[1], v[0]); // the features file has the coord swapped
                }
                
                else if(keyValPair.first == _PARSER_NORMAL_TAG){
                    std::vector<int> v = parseutils::parseCSVArray<int>(keyValPair.second);
                    mapFeature.normal = cv::Vec2i(v[0], v[1]);
            }
                lineCnt++;
            }
            return mapFeature;
        }
        
        // TODO: implement parser to populate _landmarks
        void _parseYamlFeaturesFile(){
            
            std::ifstream inFile(_landmarksFile, std::ifstream::in);
            std::string strLine;

            while (inFile.good()){
                getline(inFile, strLine);
                parseutils::KeyValuePair keyValPair = parseutils::splitKeyValue(strLine, _PARSER_DELIMITER);
                
                // parse map features
                if (keyValPair.first == _PARSER_FEATURE_TAG){
                    MapFeature mf = _initMapFeature(inFile);
                    _landmarks.insert(std::make_pair(mf.type, mf));
                }
                //std::cout << strLine << std::endl;
            }

            inFile.close();
        }

};

} // ::map

#endif // ANNOTATEDMAP_HPP_
