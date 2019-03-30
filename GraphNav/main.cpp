//
//  main.cpp
//  GraphNav
//
//  Created by Giovanni Fusco on 3/25/19.
//  Copyright © 2019 Giovanni Fusco. All rights reserved.
//

#include <iostream>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "rapidjson/document.h"
#include <fstream>
#include "include/Maps/MapManager.hpp"
#include "include/Graph.hpp"

int main(int argc, const char * argv[]) {
    std::string jsonfile = "/Users/gio/Documents/workspace/GraphNav/GraphNav/res/4thfloor.json";
    std::shared_ptr<maps::MapManager> mapManager = std::shared_ptr<maps::MapManager>(new maps::MapManager());
    int floor = 4;
    std::string mapFolder = "/Users/gio/Documents/workspace/GraphNav/GraphNav/res/maps/SKERI";
    mapManager->init(mapFolder, 4);
    navgraph::Graph navGraph(jsonfile, mapManager);
   // navGraph.plotGraph(4, true);
    while (1)
        navGraph.showClosestNodeToPoint(4);
    return 0;
}
