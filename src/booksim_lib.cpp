// $Id$

/*
 Copyright (c) 2007-2015, Trustees of The Leland Stanford Junior University
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this 
 list of conditions and the following disclaimer.
 Redistributions in binary form must reproduce the above copyright notice, this
 list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "booksim_lib.hpp"
#include "booksim.hpp"
#include "config_utils.hpp"
#include "spike.hpp"
#include "trafficmanager_spike.hpp"
#include "random_utils.hpp"
#include "network.hpp"
#include "power_module.hpp"
#include "globals.hpp"

#include <sys/time.h>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <sstream>

// Global variables to maintain BookSim2 state
static TrafficManager *trafficManager = nullptr;
static bool initialized = false;
int GetSimTime() {
  return trafficManager->getTime();
}

bool gPrintActivity;

int gK;//radix
int gN;//dimension
int gC;//concentration
int gYCount;
int gXCount;
int gNodes;

//generate nocviewer trace
bool gTrace;

ostream * gWatchOut;

vector<int> gReceiverBusyCycles{};
vector<deque<pair<int, int>>> gReceiverBuffers{};

vector<SpikeEvent> gSpikeEvents{};

void booksim_init() {
  if (initialized) {
    std::cerr << "BookSim2 is already initialized!" << std::endl;
  }
  initialized = true;

  return;
}

BookSimConfig booksim_load_config(int argc, char **argv)
{
  BookSimConfig config{};
  if ( !ParseArgs( &config, argc, argv ) ) {
    cerr << "Usage: " << argv[0] << " configfile... [param=value...]" << endl;
    return config;
  }

  string watch_out_file = config.GetStr( "watch_out" );
  gWatchOut = &cout;
  if(watch_out_file == "") {
    gWatchOut = NULL;
  } else if(watch_out_file == "-") {
    gWatchOut = &cout;
  } else {
    gWatchOut = new ofstream(watch_out_file.c_str());
  }

  /*initialize routing, traffic, injection functions
   */
  InitializeRoutingMap( config );

  gPrintActivity = (config.GetInt("print_activity") > 0);
  gTrace = (config.GetInt("viewer_trace") > 0);

  return config;
}

double booksim_run( BookSimConfig const & config)
{
  vector<Network *> net;

  int subnets = config.GetInt("subnets");
  /*To include a new network, must register the network here
   *add an else if statement with the name of the network
   */
  net.resize(subnets);
  for (int i = 0; i < subnets; ++i) {
    ostringstream name;
    name << "network_" << i;
    net[i] = Network::New( config, name.str() );

  }

  gReceiverBusyCycles.resize(gNodes, 0);
  gReceiverBuffers.resize(gNodes, deque<pair<int, int>>());

  /*tcc and characterize are legacy
   *not sure how to use them
   */

  assert(trafficManager == NULL);
  trafficManager = TrafficManagerSpike::New( config, net ) ;
  INFO("Pushing %zu spike events.\n", gSpikeEvents.size());

  for (auto &event : gSpikeEvents)
  {
    trafficManager->PushTraceEvent(event);
  }
  gSpikeEvents.clear();
  /*Start the simulation run
   */

  double total_time; /* Amount of time we've run */
  struct timeval start_time, end_time; /* Time before/after user code */
  total_time = 0.0;
  gettimeofday(&start_time, NULL);

  trafficManager->Run() ;


  gettimeofday(&end_time, NULL);
  total_time = ((double)(end_time.tv_sec) + (double)(end_time.tv_usec)/1000000.0)
            - ((double)(start_time.tv_sec) + (double)(start_time.tv_usec)/1000000.0);

  cout<<"Total run time "<<total_time<<endl;

  for (int i=0; i<subnets; ++i) {

    ///Power analysis
    if(config.GetInt("sim_power") > 0){
      Power_Module pnet(net[i], config);
      pnet.run();
    }
    delete net[i];
  }

  int cycle_count = trafficManager->getTime();
  double clock_period = config.GetFloat("clock_period");
  double run_time = cycle_count * clock_period;

  return run_time;
}

void booksim_create_processing_event( int timestep, std::pair<std::string, int> src_neuron, std::pair<int, int> src_hw, double generation_latency )
{
  SpikeEvent event = SpikeEvent::CreateProcessingEvent(timestep, src_neuron, src_hw, generation_latency);
  gSpikeEvents.push_back(event);

  return;
}

void booksim_create_spike_event( int timestep, std::pair<std::string, int> src_neuron, std::pair<int, int> src_hw, std::pair<int, int> dest_hw, double generation_latency, double processing_latency )
{
  SpikeEvent event{};
  event.event_type = SpikeEvent::Type::SPIKE_PACKET;
  event.src_neuron = src_neuron;
  event.src_hw = src_hw;
  event.dest_hw = dest_hw;
  event.generation_latency = generation_latency;
  event.processing_latency = processing_latency;

  gSpikeEvents.push_back(event);

  return;
}

void booksim_close()
{
  delete trafficManager;
  trafficManager = NULL;

  gSpikeEvents.clear();

  return;
}
