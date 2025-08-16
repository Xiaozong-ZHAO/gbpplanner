#define RLIGHTS_IMPLEMENTATION // needed to be defined once for the lights shader
#include <iostream>
#include <Utils.h>
#include <DArgs.h>
#include <Globals.h>
#include <Simulator.h>

Globals globals;

int main(int argc, char *argv[]) {
    std::cout << "Running GTSAM centralized optimization test..." << std::endl;
    
    srand((int)globals.SEED);                                   // Initialize random seed   
    DArgs::DArgs dargs(argc, argv);                             // Parse config file argument --cfg <file.json>
    if (globals.parse_global_args(dargs)) return EXIT_FAILURE;  
    
    Simulator* sim = new Simulator();       // Initialize the simulator
    globals.RUN = true;
    
    std::cout << "Formation: " << globals.FORMATION << ", Robots: " << globals.NUM_ROBOTS << std::endl;
    
    while (globals.RUN) {
        sim->eventHandler();                // Capture keypresses or mouse events             
        sim->createOrDeleteRobotsGTSAM();   // Create GTSAM robots instead of GBP robots
        sim->timestepGTSAM();               // Use GTSAM timestep for centralized optimization
        sim->draw();
    }

    delete sim;
    return 0;
}
