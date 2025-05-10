#include "booksim_lib.hpp"
#include "booksim.hpp"
#include "config_utils.hpp"
#include "trafficmanager_spike.hpp"
#include "random_utils.hpp"
#include "network.hpp"
#include "power_module.hpp"

#include <sys/time.h>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <sstream>

// Global variables to maintain BookSim2 state
static TrafficManager *trafficManager = nullptr;
static bool initialized = false;

bool gPrintActivity;

int gK;//radix
int gN;//dimension
int gC;//concentration
int gYCount;
int gXCount;

int gNodes;

//extern "C" {

void booksim_init(int argc, char **argv) {
    BookSimConfig config;
  if (initialized) {
    std::cerr << "BookSim2 is already initialized!" << std::endl;
    return;
  }

  if ( !ParseArgs( &config, argc, argv ) ) {
    cerr << "Usage: " << argv[0] << " configfile... [param=value...]" << endl;
    return;
  }

  /*initialize routing, traffic, injection functions
   */
  InitializeRoutingMap( config );

  gPrintActivity = (config.GetInt("print_activity") > 0);
  gTrace = (config.GetInt("viewer_trace") > 0);

  string watch_out_file = config.GetStr( "watch_out" );
  gWatchOut = &cout;
  /*
  if(watch_out_file == "") {
    gWatchOut = NULL;
  } else if(watch_out_file == "-") {
    gWatchOut = &cout;
  } else {
    gWatchOut = new ofstream(watch_out_file.c_str());
  }
  */

  /*configure and run the simulator
   */
}

bool booksim_run( BookSimConfig const & config )
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

  /*Start the simulation run
   */

  double total_time; /* Amount of time we've run */
  struct timeval start_time, end_time; /* Time before/after user code */
  total_time = 0.0;
  gettimeofday(&start_time, NULL);

  bool result = trafficManager->Run() ;


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

  delete trafficManager;
  trafficManager = NULL;

  return result;
}

//} // extern "C"