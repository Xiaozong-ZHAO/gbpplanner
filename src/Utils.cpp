/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <Utils.h>
#include <Globals.h>
extern Globals globals;

/**************************************************************************************/
// Draw the Time and FPS, as well as the help box
/**************************************************************************************/
void draw_info(uint32_t time_cnt){
    // 现有代码保持不变...
    
    if (globals.SIM_MODE==Help){
        int info_box_h = 500, info_box_w = 500;
        int info_box_x = globals.SCREEN_SZ/2 - info_box_w/2, info_box_y = globals.SCREEN_SZ/2 - info_box_h/2;
        DrawRectangle( info_box_x, info_box_y, info_box_w, info_box_h, Fade(SKYBLUE, 0.5f));
        DrawRectangleLines( info_box_x, info_box_y, info_box_w, info_box_h, BLACK);

        int offset = 40;
        std::vector<std::string> texts{
            "Esc : \t\t\t\t Exit Simulation",
            "H : \t\t\t\t\t\t Close Help",
            "SPACE : \t Camera Transition",
            "ENTER : \t Run/Pause Simulation",
            "P : \t\t\t\t\t\t Toggle Planned paths",
            "R : \t\t\t\t\t\t Toggle Connected Robots",
            "W : \t\t\t\t\t\t Toggle Waypoints",
            "V : \t\t\t\t\t\t Toggle Robot Velocities",  // 新增
            ""         ,
            "Mouse Wheel Scroll : Zoom",
            "Mouse Wheel Drag : Pan",
            "Mouse Wheel Drag + SHIFT : Rotate",
        };

        for (int t=0; t<texts.size(); t++){
            DrawText(texts[t].c_str(), info_box_x + 10, info_box_y + (t+1)*offset, 20, BLACK);
        }
    }
}


