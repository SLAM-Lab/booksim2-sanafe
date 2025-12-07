#include <iostream>
#include "trafficmanager.hpp"
#include "globals.hpp"

int SimContext::getSimTime() {
    return trafficManager->getTime();
}

Stats * SimContext::getStats(const std::string & name) {
    Stats* test =  trafficManager->getStats(name);
    if(test == 0){
        cout<<"warning statistics "<<name<<" not found"<<endl;
    }
    return test;
}

void SimContext::init(const Configuration& config) {
    gK = config.GetInt("k");
    SimContext::get().gN = config.GetInt("n");
    gC = config.GetInt("c");
    gNodes = config.GetInt("nodes");
}

SimContext &SimContext::get()
{
    static thread_local SimContext instance;
    return instance;
}