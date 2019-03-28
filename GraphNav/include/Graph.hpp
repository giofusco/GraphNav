//
//  Graph.hpp
//  GraphNav
//
//  Created by Giovanni Fusco on 3/26/19.
//  Copyright © 2019 Giovanni Fusco. All rights reserved.
//

#ifndef Graph_h
#define Graph_h

#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "rapidjson/document.h"
#include <fstream>
#include <map>

namespace navgraph{
    

class Graph {
    
public:
    enum NodeType { Control = 0, Destination = 1, Link = 2 };
    
    struct Edge{
        float length; // length of the edge in pixels in u,v coordinates
        float angleDeg; // orientation of the edge from source to destination
        float a;  //line coefficients: ax + by = c
        float b;
        float c;
    };
    
    struct Node{
        //  int id;
        NodeType type;
        cv::Point2f positionUV;
        int floor;
        std::string label;
        bool isDoor;
        std::string comments;
        std::map<int, Edge> edges;
    };
    
    
    Graph() { ; }
    Graph(std::string jsonFileName, std::shared_ptr<maps::MapManager> mapManager){
        rapidjson::Document document;
        std::ifstream infile(jsonFileName);
        std::string json((std::istreambuf_iterator<char>(infile)),
                         std::istreambuf_iterator<char>());
        document.Parse(json.c_str());
        _parseGraphMatrices(document);
        _parseNodes(document);
        _computeLinesCoeffs();
        _mapManager = mapManager;
    }
    
    void _computeLinesCoeffs(){
        for (auto &n : _nodes){
            cv::Point2f n1Pos = toIJ(n.second.positionUV);
            for (auto &e : _nodes[n.first].edges){
                cv::Point2f n2Pos = toIJ(_nodes[e.first].positionUV);
            }
        }
    }
    
    cv::Mat plotGraph(int floor, bool pause = false){
        // plot graph relative to the specified floor
        cv::Mat map = _mapManager->getWallsImageRGB();
        for (const auto& n : _nodes ){
            if (n.second.floor == floor)
                cv::circle(map, n.second.positionUV, 3, getNodeColor(n.second.type));
        }
        cv::imshow("Graph floor " + std::to_string(floor), map);
        if (pause)
            cv::waitKey(-1);
        return map;
    }
    
    inline cv::Point toIJ(cv::Point uv){ //sets origin in lower left corner
        return cv::Point(uv.x, _mapManager->getMapSizePixels().height - uv.y);
    }
    
    cv::Scalar getNodeColor(NodeType type){
        switch (type){
            case NodeType::Control:
                return cv::Scalar(0,255,0);
            case NodeType::Destination:
                return cv::Scalar(255,0,0);
            case NodeType::Link:
                return cv::Scalar(0,0,255);
            default:
                return cv::Scalar(0,255,0);
        }
    }
    
    
    void showClosestNodeToPoint(int floor, bool checkWalls = false){
        cv::Mat map = plotGraph(floor);
        cv::namedWindow("graph");
        cv::Point pt;
        cv::setMouseCallback( "graph", onMouse, &pt );
        
        cv::imshow("graph", map);
        cv::waitKey(0);
        
        int id = findClosestNodeId(pt, floor, checkWalls);
        if (id > 0){
            cv::circle(map, _nodes[id].positionUV, 3, cv::Scalar(0,255,255));
            cv::imshow("Closest node", map);
            cv::waitKey(-1);
        }
    }
    
    // for debugging
    static void onMouse( int event, int x, int y, int, void* ptr)
    {
        if( event != cv::EVENT_LBUTTONDOWN )
            return;
        cv::Point* p = (cv::Point*)ptr;
        p->x = x;
        p->y = y;
        std::cerr << x << "," << y << "\n";
    }
    
    int findClosestNodeId(cv::Point2f pos, int floor, bool checkWalls = false){
        int id = -1;
        float minDist = 1e6;
        float d;
        for (const auto& n : _nodes){
            if (n.second.floor == floor){
                cv::Point diff = pos - n.second.positionUV;
                d = (diff.x*diff.x + diff.y*diff.y);
                if (d < minDist){
                    minDist = d;
                    id = n.first;
                }
            }
                
        }
        return id;
    }
    
private:
    cv::Mat _weights;
    cv::Mat _angles;
    std::map<int, Graph::Node> _nodes;
    std::shared_ptr<maps::MapManager> _mapManager;
    
    // load weights and angle matrices
    void _parseGraphMatrices(const rapidjson::Document &document){
        const rapidjson::Value& w = document["weights"];
        const rapidjson::Value& a = document["angles"];
        _weights = cv::Mat::zeros(a.Size(), a.Size(), CV_32F);
        _angles = cv::Mat::zeros(a.Size(), a.Size(), CV_32F);
        for (rapidjson::SizeType i = 0; i < a.Size(); i++){
            float* Wi = _weights.ptr<float>(i);
            float* Ai = _angles.ptr<float>(i);
            const rapidjson::Value& ai = a[i];
            const rapidjson::Value& wi = w[i];
            for (rapidjson::SizeType j = 0; j < ai.Size(); j++){
                Wi[j] = wi[j].GetFloat();
                Ai[j] = ai[j].GetFloat();
            }
        }
    }
    
    void _parseNodes(const rapidjson::Document &document){
        const auto& nodes = document["nodes"][0].GetObject();
        for(rapidjson::Value::ConstMemberIterator it=nodes.MemberBegin(); it != nodes.MemberEnd(); it++) {
            int nodeId = it->value["id"].GetInt();
            Node node;
            std::string ntype = it->value["type"].GetString();
            if (ntype.compare("control") == 0)
                node.type = NodeType::Control;
            else if (ntype.compare("destination") == 0)
                 node.type = NodeType::Destination;
            else if (ntype.compare("link") == 0)
                node.type = NodeType::Link;
            
//            node.type = it->value["type"].GetString();
            node.floor = std::stoi(it->value["floor"].GetString());
            node.label = it->value["label"].GetString();
            node.isDoor = it->value["isDoor"].GetInt();
            node.comments = it->value["comments"].GetString();
            
            const auto& pos = it->value["position"].GetArray();
            node.positionUV = cv::Point2f(pos[0].GetFloat(), pos[1].GetFloat());
            
            float* wi = _weights.ptr<float>(nodeId-1);
            float* ai = _angles.ptr<float>(nodeId-1);
            
            for (int j = 0; j < _weights.cols; j++){
                if (wi[j] > 0){
                    Edge e = { .length = wi[j], .angleDeg = ai[j]};
                    node.edges.insert({j+1, e});
                }
            }
            _nodes.insert({it->value["id"].GetInt(), node});
        
        }
    }
};

#endif /* Graph_h */

} // end navgraph namespace
