// trafficmanager_spike.cpp implementation

#include "trafficmanager_spike.hpp"
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <map>
#include <queue>
#include <string>
#include <sstream>

using namespace std;

#include "trafficmanager_spike.hpp"
#include "random_utils.hpp"
#include "misc_utils.hpp"
#include "booksim.hpp"
#include "booksim_config.hpp"
#include "trafficmanager.hpp"
#include "batchtrafficmanager.hpp"
#include "vc.hpp"
#include "packet_reply_info.hpp"

TrafficManagerSpike::TrafficManagerSpike(const Configuration& config, const vector<Network*>& net)
    : TrafficManager(config, net) {
    trace_file = config.GetStr("trace_file");
    clock_period = config.GetFloat("clock_period");

    if (!OpenTrace()) {
        cerr << "Error: Could not open trace file " << trace_file << endl;
        exit(-1);
    }

    //_pe_rx_busy = std::vector<bool>(_nodes, false);
    _pe_tx_busy = std::vector<bool>(_nodes, false);
    _received_flits = std::vector<std::queue<int>>(_nodes);
    _message_count = std::vector<long long int>(_nodes, 0ULL);
    //flits = std::vector<map<int, Flit *>>(_subnets);
}

TrafficManagerSpike::~TrafficManagerSpike() {
    if (trace_stream.is_open()) {
        trace_stream.close();
    }
}

TrafficManagerSpike * TrafficManagerSpike::New(Configuration const & config,
                                     vector<Network *> const & net)
{
    TrafficManagerSpike * result = NULL;
    string sim_type = config.GetStr("sim_type");
    if(sim_type == "latency") {
        return new TrafficManagerSpike(config, net);
    } else if (sim_type == "throughput") {
        cerr << "Unsupported simulation type: " << sim_type << endl;
    } else if(sim_type == "batch") {
        cerr << "Unsupported simulation type: " << sim_type << endl;
    } else {
        cerr << "Unknown simulation type: " << sim_type << endl;
    }
    return result;
}

bool TrafficManagerSpike::OpenTrace() {
    std::cout << "Opening trace file\n";
    trace_stream.open(trace_file.c_str());
    if (!trace_stream.is_open()) {
        return false;
    }

    // Skip header
    std::string header;
    getline(trace_stream, header);
    return true;
}

bool TrafficManagerSpike::_InjectionPossible(const int source, const int dest,
        const int subnet)
{
    // Check if injection is possible by verifying buffer state
    BufferState * const dest_buf = _buf_states[source][subnet];

    // Create the packet's head flit to check routing
    Flit * f = Flit::New();
    f->src = source;
    f->dest = dest;
    f->head = true;

    // Try to find an available VC
    OutputSet route_set;
    _rf(NULL, f, -1, &route_set, true);
    set<OutputSet::sSetElement> const & os = route_set.GetSet();
    assert(os.size() == 1);
    OutputSet::sSetElement const & se = *os.begin();

    int vc_start = se.vc_start;
    int vc_end = se.vc_end;
    bool injection_possible = false;

    INFO("buffer occupancy:%d\n", dest_buf->Occupancy());
    // Check if any VC is available and has space
    for(int vc = vc_start; vc <= vc_end; ++vc) {
        INFO("Buffer state before injection - VC:%d occupied:%d available:%d\n",
                            vc, dest_buf->IsFullFor(vc), dest_buf->IsAvailableFor(vc));
        if(dest_buf->IsAvailableFor(vc) && !dest_buf->IsFullFor(vc)) {
            INFO("injection is possible at PE:%d vc%d\n", source, vc);
            injection_possible = true;
            break;
        } else {
            INFO("vc:%d is full\n", vc);
        }
    }

    INFO("injection:%d for %d->%d\n", injection_possible, source, dest);

    f->Free();
    return injection_possible;
}


void TrafficManagerSpike::_Inject() {
    INFO("injecting messages:%d\n", _time);

    // Look at all cores and all pending packets, see if any are ready to send
    //  another packet then run the following code
    //    if core has a message ready to go (i.e. processing time=0, but is busy)
    //      if the network is ready:
    //          send the message
    //          set busy=false
    //    if no message was ready i.e. busy == false:
    //      get the next message to inject and update processing time
    //      busy = true
    for (int n = 0; n < _nodes; ++n) {
        if ( _tx_processing_cycles_left[n] > 0 ) {
            --_tx_processing_cycles_left[n];
            INFO("core %d tx cycles_left:%lld\n",
                    n, _tx_processing_cycles_left[n]);
        }
    }
    // TODO: somehow predetermine which subnet packets should go to. That means
    //  probably restructuring this code to iterate over nodes and send to
    //  whatever subnet is designated. If no subnet is designated, we could in
    //  future find the available subnet
    for ( int n = 0; n < _nodes; ++n ) {
        if ( _tx_processing_cycles_left[n] <= 0  && !pending_events[n].empty()) {
            // Busy flag is set, which means the next message just finished
            //  being generated and we can fetch the next spike message
            if ( _pe_tx_busy[n] ) {
                SpikeEvent next_event = pending_events[n].front();
                int dest_core = next_event.dest_hw.first * CORES_PER_TILE +
                        next_event.dest_hw.second;
                INFO("Ready to inject next spike packet at PE:%d\n", n);

                constexpr bool dyn_subnet_alloc = false;
                int subnet;
                if (dyn_subnet_alloc) { // TODO: set in config file
                    for (subnet = 0; subnet < _subnets; ++subnet) {
                        if (_InjectionPossible(n, dest_core, subnet)) {
                            int processing_cycles = CyclesFromTime(
                            next_event.processing_latency);
                            // Network is ready so send the message
                            INFO("Injecting packet from core %d into net at t:%d\n",
                                n, _time);
                            int packet_id = _GeneratePacket(n, 1, 0, _time,
                                    dest_core, processing_cycles, subnet);
                            INFO("New flit fid:%d src:%d dest:%d\n", packet_id, n,
                                    dest_core);
                            _flit_processing_cycles[packet_id] = processing_cycles;
                            INFO("Flit fid:%d will have %d processing cycles\n",
                                    packet_id, processing_cycles);
                            // Remove the pending event for this core
                            pending_events[n].pop();
                            _pe_tx_busy[n] = false;
                            break;
                        }
                    }
                } else {
                    subnet = _message_count[n] % _subnets;
                    // If core has processed neurons and generated a message
                    if (_InjectionPossible(n, dest_core, subnet)) {
                        int processing_cycles = CyclesFromTime(
                            next_event.processing_latency);
                        // Network is ready so send the message
                        INFO("Injecting packet from core %d into net at t:%d\n",
                            n, _time);
                        int packet_id = _GeneratePacket(n, 1, 0, _time,
                                dest_core, processing_cycles, subnet);
                        INFO("Node %d sent %lld messages.\n", n, _message_count[n]);
                        ++(_message_count[n]);
                        INFO("New flit fid:%d src:%d dest:%d\n", packet_id, n,
                                dest_core);
                        _flit_processing_cycles[packet_id] = processing_cycles;
                        INFO("Flit fid:%d will have %d processing cycles\n",
                                packet_id, processing_cycles);
                        // Remove the pending event for this core
                        pending_events[n].pop();
                        _pe_tx_busy[n] = false;
                    } else {
                        INFO("Could not inject packet from core %d into net\n", n);
                    }
                }
            }
            if (!_pe_tx_busy[n] && !pending_events[n].empty()) {
                // No message currently ready, so get the next one
                INFO("Getting next event (spike/processing)\n");
                SpikeEvent next_event = pending_events[n].front();
                int injection_cycles = CyclesFromTime(
                        next_event.generation_latency);
                if (next_event.event_type ==  SpikeEvent::Type::PROCESSING_ONLY) {
                    INFO("Dummy event now handled for core %d cycles:%d\n", n,
                            injection_cycles);
                    pending_events[n].pop();
                    assert(pending_events[n].empty());
                } else {
                    INFO("Getting next spike packet for core %d cycles:%d\n",
                            n, injection_cycles);
                }
                _tx_processing_cycles_left[n] = injection_cycles;
                _pe_tx_busy[n] = true;
            }
        }
    }
}

int TrafficManagerSpike::CyclesFromTime(double time_seconds) {
    return static_cast<int>(time_seconds / clock_period + 0.5);
}

void TrafficManagerSpike::ReadNextTrace() {
    std::string line;
    while (getline(trace_stream, line)) {
        std::stringstream ss(line);
        std::string token;

        // Parse CSV format
        int timestep;
        std::pair<int, int> src_neuron, src_hw;
        std::string dest_hw_str;
        double generation_latency;

        // Parse basic fields
        getline(ss, token, ','); timestep = stoi(token);

        // Parse src_neuron
        getline(ss, token, ',');
        size_t dot_pos = token.find('.');
        src_neuron = std::make_pair(
            stoi(token.substr(0, dot_pos)),
            stoi(token.substr(dot_pos + 1))
        );

        // Parse src_hw
        getline(ss, token, ',');
        dot_pos = token.find('.');
        src_hw = std::make_pair(
            stoi(token.substr(0, dot_pos)),
            stoi(token.substr(dot_pos + 1))
        );

        // Parse dest_hw
        getline(ss, token, ',');
        dest_hw_str = token;

        // Skip hops and spikes
        getline(ss, token, ','); // hops
        getline(ss, token, ','); // spikes

        // Get generation latency
        getline(ss, token, ',');
        generation_latency = stod(token);

        if (dest_hw_str == "x.x") {
            // This is a spike-processing-only event
            SpikeEvent event = SpikeEvent::CreateProcessingEvent(
                timestep,
                src_neuron,
                src_hw,
                generation_latency
            );
            INFO("Adding dummy processing event, generation_latency:%lf\n", generation_latency);
            int src_core = event.src_hw.first * CORES_PER_TILE + event.src_hw.second;
            pending_events[src_core].push(event);
        } else {
            // This is a regular spike packet event
            SpikeEvent event;
            event.event_type = SpikeEvent::Type::SPIKE_PACKET;
            event.timestep = timestep;
            event.src_neuron = src_neuron;
            event.src_hw = src_hw;

            // Parse destination hardware address
            dot_pos = dest_hw_str.find('.');
            event.dest_hw = std::make_pair(
                stoi(dest_hw_str.substr(0, dot_pos)),
                stoi(dest_hw_str.substr(dot_pos + 1))
            );

            // Parse remaining fields
            event.generation_latency = generation_latency;
            getline(ss, token, ','); event.network_latency = stod(token);
            getline(ss, token, ','); event.processing_latency = stod(token);
            getline(ss, token, ','); event.blocking_latency = stod(token);

            INFO("Adding spike event: generation_latency:%e processing_latency:%e\n",
                    event.generation_latency, event.processing_latency);
            int src_core = event.src_hw.first * CORES_PER_TILE + event.src_hw.second;
            pending_events[src_core].push(event);
            INFO("Pushed event to PE:%d Now %zu pending spike events\n",
                    src_core, pending_events[src_core].size());
        }
    }
}

bool TrafficManagerSpike::_NeuronProcessing() {
    // Check to see if any core is still processing its neurons
    return std::any_of(_tx_processing_cycles_left.begin(),
            _tx_processing_cycles_left.end(),
            [](std::pair<const int, const long long int> src_core_cycles_pair){
                return (src_core_cycles_pair.second != 0);
            });
}

bool TrafficManagerSpike::_Pending() {
    // Use std::any_of to check if any PE's event queue is not empty
    return std::any_of(pending_events.begin(), pending_events.end(),
        [](std::pair<const int,
            const std::queue<SpikeEvent>> src_core_pending_q_pair) {
            return !src_core_pending_q_pair.second.empty();
    });
}

bool TrafficManagerSpike::_SingleSim() {
    _Step();

    // Check if we need to read more traces
    if (!_Pending()) {
        ReadNextTrace();
    }

    constexpr int max_time = 100000; // TODO: figure better way to limit time
    while ( _Pending() && (_time < max_time) )
    {
        _Step();
    }

    // Check if simulation should continue
    if (!_Pending()) {
        INFO("Simulation finished successfully\n");
        return true;  // Simulation complete
    }

    INFO("Error: not all spikes pushed\n");
    return false;
}

void TrafficManagerSpike::_Step()
{
    INFO("*** Stepping sim t:%d***\n", _time);
    bool flits_in_flight = false;
    for(int c = 0; c < _classes; ++c) {
        flits_in_flight |= !_total_in_flight_flits[c].empty();
    }
    if(!flits_in_flight && !_Pending() &&
            (_NeuronProcessing() ||  !_MessagesBeingReceived())) {
        // There are no more flits in flight, but we still need to wait for the
        //  remaining flits to be processed, or the cores to process their final
        //  neurons
        INFO("Stepping, no in-flight flits but neurons/messages being processed\n");

        // Still update the neuron processing / Tx counters, in case there
        //  is processing that occurs after the last message has been sent by
        //  a core
        for (int n = 0; n < _nodes; ++n) {
            if ( _tx_processing_cycles_left[n] > 0 ) {
                --_tx_processing_cycles_left[n];
                INFO("core %d tx cycles_left:%lld\n",
                        n, _tx_processing_cycles_left[n]);
            }
        }

        _time++;
        return;
    }
    /*
    if(flits_in_flight && (_deadlock_timer++ >= _deadlock_warn_timeout)){
        _deadlock_timer = 0;
        cout << "WARNING: Possible network deadlock.\n";
    }
    */

    // Update rx processing
    /*
    for (int subnet = 0; subnet < _subnets; ++subnet) {
        for (int n = 0; n < _nodes; ++n) {
            if (_rx_processing_cycles_left[n] > 0) {
                --_rx_processing_cycles_left[n];
                if (_rx_processing_cycles_left[n] == 0) {
                    INFO("PE:%d received and processed flit fid:%d now ready to receive more messages\n",
                            n, _received_flits[n].front());
                    _received_flits[n].pop();
                    _pe_rx_busy[n] = false;
                }
                //    INFO("core %d now has %lld rx cycles left\n", dest,
                //            _rx_processing_cycles_left[dest]);
            }
        }
    }
    */

    vector<map<int, Flit *> > flits(_subnets);

    for ( int subnet = 0; subnet < _subnets; ++subnet ) {
        for ( int n = 0; n < _nodes; ++n ) {
            Flit * const f = _net[subnet]->ReadFlit( n );
            if ( f != nullptr ) {
                INFO("Reading flit fid:%d for PE:%d at time:%d\n",
                        f->pid, n, _time);
                _received_flits[n].push(f->pid);
                // TODO: I changed the mechanism I used - this code and variables
                //  are now redundant
                //_pe_rx_busy[n] = true;
                //_rx_processing_cycles_left[n] =
                //        _flit_processing_cycles[f->pid];
                //INFO("Marking PE:%d rx busy with fid:%d for %lld cycles\n",
                //        n, f->pid, _flit_processing_cycles[f->pid]);
                if(f->watch) {
                    *gWatchOut << GetSimTime() << " | "
                            << "node" << n << " | "
                            << "Ejecting flit " << f->id
                            << " (packet " << f->pid << ")"
                            << " from VC " << f->vc
                            << "." << endl;
                }

                flits[subnet].insert(make_pair(n, f));
                if((_sim_state == warming_up) || (_sim_state == running)) {
                    ++_accepted_flits[f->cl][n];
                    if(f->tail) {
                        ++_accepted_packets[f->cl][n];
                    }
                }
            }

            //INFO("Reading credit for PE:%d\n", n);
            Credit * const c = _net[subnet]->ReadCredit( n );
            if ( c ) {
#ifdef TRACK_FLOWS
                for(set<int>::const_iterator iter = c->vc.begin(); iter != c->vc.end(); ++iter) {
                    int const vc = *iter;
                    assert(!_outstanding_classes[n][subnet][vc].empty());
                    int cl = _outstanding_classes[n][subnet][vc].front();
                    _outstanding_classes[n][subnet][vc].pop();
                    assert(_outstanding_credits[cl][subnet][n] > 0);
                    --_outstanding_credits[cl][subnet][n];
                }
#endif
                INFO("Buffer state before ProcessCredit - VC:%d available:%d\n",
                        *c->vc.begin(), _buf_states[n][subnet]->IsAvailableFor(*(c->vc.begin())));
                INFO("occupancy before:%d\n", _buf_states[n][subnet]->Occupancy());
                _buf_states[n][subnet]->ProcessCredit(c);
                INFO("Buffer state after ProcessCredit - VC:%d available:%d\n",
                        *c->vc.begin(), _buf_states[n][subnet]->IsAvailableFor(*(c->vc.begin())));
                INFO("occupancy after:%d\n", _buf_states[n][subnet]->Occupancy());
                c->Free();
            }
        }
        _net[subnet]->ReadInputs( );
    }

    if ( !_empty_network ) {
        INFO("calling inject\n");
        _Inject();
    }

    // Inject all flits ready to be sent
    // 1. Iterating through all subnets and nodes
    // 2. For each node, check if there are partial packets (flits that haven't
    //     been injected yet)
    // 3. If there is a partial packet waiting, then try to find an available
    //     virtual channel
    //    a. Check the last used channel to see if available
    //    b. Remove the flit from the partial packet list
    //    c. Set flit priority
    //    d. Log info on the flit
    //    e. If there's any remaining partial packets pass the virtual channel
    //        back to the next flit in the list
    for(int subnet = 0; subnet < _subnets; ++subnet) {

        for(int n = 0; n < _nodes; ++n) {

            Flit * f = NULL;

            BufferState * const dest_buf = _buf_states[n][subnet];

            int const last_class = _last_class[n][subnet];

            int class_limit = _classes;

            if(_hold_switch_for_packet) {
                list<Flit *> const & pp = _partial_packets[n][last_class];
                if(!pp.empty() && !pp.front()->head &&
                   !dest_buf->IsFullFor(pp.front()->vc)) {
                    f = pp.front();
                    INFO("partial packet front() returned fid:%d\n", f->pid);
                    assert(f->vc == _last_vc[n][subnet][last_class]);

                    // if we're holding the connection, we don't need to check that class
                    // again in the for loop
                    --class_limit;
                }
            }

            for(int i = 1; i <= class_limit; ++i) {

                int const c = (last_class + i) % _classes;

                list<Flit *> const & pp = _partial_packets[n][c];
                /*
                INFO("partial packets PE:%d class:%d len:%zu\n",
                        n, c, pp.size());
                INFO("** flits: **\n");
                for (auto f: pp) {
                    INFO("\tfid:%d\n", f->pid);
                }
                */

                if(pp.empty()) {
                    //INFO("pp empty, continuing\n");
                    continue;
                }

                Flit * const cf = pp.front();
                assert(cf);
                assert(cf->cl == c);
                INFO("current flit fid:%d\n", cf->pid);

                if(cf->subnetwork != subnet) {
                    continue;
                }

                if(f && (f->pri >= cf->pri)) {
                    continue;
                }

                if(cf->head && cf->vc == -1) { // Find first available VC

                    OutputSet route_set;
                    _rf(NULL, cf, -1, &route_set, true);
                    set<OutputSet::sSetElement> const & os = route_set.GetSet();
                    assert(os.size() == 1);
                    OutputSet::sSetElement const & se = *os.begin();
                    assert(se.output_port == -1);
                    int vc_start = se.vc_start;
                    int vc_end = se.vc_end;
                    int vc_count = vc_end - vc_start + 1;
                    if(_noq) {
                        assert(_lookahead_routing);
                        const FlitChannel * inject = _net[subnet]->GetInject(n);
                        const Router * router = inject->GetSink();
                        assert(router);
                        int in_channel = inject->GetSinkPort();

                        // NOTE: Because the lookahead is not for injection, but for the
                        // first hop, we have to temporarily set cf's VC to be non-negative
                        // in order to avoid seting of an assertion in the routing function.
                        cf->vc = vc_start;
                        _rf(router, cf, in_channel, &cf->la_route_set, false);
                        cf->vc = -1;

                        INFO("PE:%d generating lookahead routing info for flit fid:%d\n", n, cf->pid);
                        if(cf->watch) {
                            //*gWatchOut << GetSimTime() << " | "
                            //           << "node" << n << " | "
                            //           << "Generating lookahead routing info for flit " << cf->id
                            //           << " (NOQ)." << endl;
                        }
                        set<OutputSet::sSetElement> const sl = cf->la_route_set.GetSet();
                        assert(sl.size() == 1);
                        int next_output = sl.begin()->output_port;
                        vc_count /= router->NumOutputs();
                        vc_start += next_output * vc_count;
                        vc_end = vc_start + vc_count - 1;
                        assert(vc_start >= se.vc_start && vc_start <= se.vc_end);
                        assert(vc_end >= se.vc_start && vc_end <= se.vc_end);
                        assert(vc_start <= vc_end);
                    }
                    //if(cf->watch) {
                        INFO("finding output vc for flit fid:%d\n", cf->pid);
                        //*gWatchOut << GetSimTime() << " | " << FullName() << " | "
                          //         << "Finding output VC for flit " << cf->id
                          //         << ":" << endl;
                    //}
                    for(int i = 1; i <= vc_count; ++i) {
                        int const lvc = _last_vc[n][subnet][c];
                        int const vc =
                            (lvc < vc_start || lvc > vc_end) ?
                            vc_start :
                            (vc_start + (lvc - vc_start + i) % vc_count);
                        assert((vc >= vc_start) && (vc <= vc_end));
                        if(!dest_buf->IsAvailableFor(vc)) {
                            //if(cf->watch) {
                                INFO("output vc:%d is busy\n", vc);
                               // *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                                //           << "  Output VC " << vc << " is busy." << endl;
                            //}
                        } else {
                            if(dest_buf->IsFullFor(vc)) {
                                //if(cf->watch) {
                                    INFO("output vc:%d is full\n", vc);
                                  //  *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                                   //            << "  Output VC " << vc << " is full." << endl;
                                //}
                            } else {
                                //if(cf->watch) {
                                INFO("selected output vc:%d\n", vc);
                                //    *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                                //               << "  Selected output VC " << vc << "." << endl;
                                //}
                                cf->vc = vc;
                                break;
                            }
                        }
                    }
                }

                if(cf->vc == -1) {
                    //if(cf->watch) {
                        //*gWatchOut << GetSimTime() << " | " << FullName() << " | "
                        //           << "No output VC found for flit " << cf->id
                        //           << "." << endl;
                        INFO("no output vc found for flit fid:%d'n", cf->pid);
                    //}
                } else {
                    if(dest_buf->IsFullFor(cf->vc)) {
                        //if(cf->watch) {
                           // *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                           //            << "Selected output VC " << cf->vc
                           //            << " is full for flit " << cf->id
                           //            << "." << endl;
                           INFO("selected output vc:%d is full for flid fid:%d\n", cf->vc, cf->pid);
                        //}
                    } else {
                        f = cf;
                    }
                }
            }

            if(f) {

                assert(f->subnetwork == subnet);

                int const c = f->cl;

                if(f->head) {

                    if (_lookahead_routing) {
                        if(!_noq) {
                            const FlitChannel * inject = _net[subnet]->GetInject(n);
                            const Router * router = inject->GetSink();
                            assert(router);
                            int in_channel = inject->GetSinkPort();
                            _rf(router, f, in_channel, &f->la_route_set, false);
                            if(f->watch) {
                                *gWatchOut << GetSimTime() << " | "
                                           << "node" << n << " | "
                                           << "Generating lookahead routing info for flit " << f->id
                                           << "." << endl;
                            }
                        } else if(f->watch) {
                            *gWatchOut << GetSimTime() << " | "
                                       << "node" << n << " | "
                                       << "Already generated lookahead routing info for flit " << f->id
                                       << " (NOQ)." << endl;
                        }
                    } else {
                        f->la_route_set.Clear();
                    }

                    INFO("fid:%d taking buffer %d\n", f->pid, f->vc);

                    INFO("Buffer state before TakeBuffer - VC:%d available:%d\n",
                            f->vc, dest_buf->IsAvailableFor(f->vc));
                    INFO("occupancy before:%d\n", dest_buf->Occupancy());
                    dest_buf->TakeBuffer(f->vc);
                    INFO("Buffer state after TakeBuffer - VC:%d available:%d\n",
                            f->vc, dest_buf->IsAvailableFor(f->vc));
                    INFO("occupancy after:%d\n", dest_buf->Occupancy());
                    _last_vc[n][subnet][c] = f->vc;
                }

                _last_class[n][subnet] = c;

                _partial_packets[n][c].pop_front();

#ifdef TRACK_FLOWS
                ++_outstanding_credits[c][subnet][n];
                _outstanding_classes[n][subnet][f->vc].push(c);
#endif

                INFO("sending flit fid:%d\n", f->id);
                INFO("Buffer state before SendingFlit - VC:%d available:%d\n",
                            f->vc, dest_buf->IsAvailableFor(f->vc));
                INFO("occupancy before SendingFlit:%d\n", dest_buf->Occupancy());
                dest_buf->SendingFlit(f);
                INFO("Buffer state after SendingFlit - VC:%d available:%d\n",
                            f->vc, dest_buf->IsAvailableFor(f->vc));
                INFO("occupancy after SendingFlit:%d\n", dest_buf->Occupancy());

                if(_pri_type == network_age_based) {
                    f->pri = numeric_limits<int>::max() - _time;
                    assert(f->pri >= 0);
                }

                if(f->watch) {
                    *gWatchOut << GetSimTime() << " | "
                               << "node" << n << " | "
                               << "Injecting flit " << f->id
                               << " into subnet " << subnet
                               << " at time " << _time
                               << " with priority " << f->pri
                               << "." << endl;
                }
                f->itime = _time;

                // Pass VC "back"
                INFO("pass VC back fid:%d PE:%d class:%d\n", f->pid, n, c);
                if(!_partial_packets[n][c].empty() && !f->tail) {
                    Flit * const nf = _partial_packets[n][c].front();
                    nf->vc = f->vc;
                }

                if((_sim_state == warming_up) || (_sim_state == running)) {
                    ++_sent_flits[c][n];
                    if(f->head) {
                        ++_sent_packets[c][n];
                        INFO("sent packet fid:%d\n", f->pid);
                    }
                }

#ifdef TRACK_FLOWS
                ++_injected_flits[c][n];
#endif

                INFO("occupancy before WriteFlit:%d\n", dest_buf->Occupancy());
                INFO("writing flit fid:%d\n", f->id);
                _net[subnet]->WriteFlit(f, n);
                INFO("occupancy after WriteFlit:%d\n", dest_buf->Occupancy());
            }
        }
    }

    for(int subnet = 0; subnet < _subnets; ++subnet) {
        for(int n = 0; n < _nodes; ++n) {
            map<int, Flit *>::const_iterator iter = flits[subnet].find(n);
            if(iter != flits[subnet].end()) {
                Flit * const f = iter->second;
                INFO("Flit fid:%d received at PE:%d\n", f->pid, n);

                f->atime = _time;
                if(f->watch) {
                    *gWatchOut << GetSimTime() << " | "
                                << "node" << n << " | "
                                << "Injecting credit for VC " << f->vc
                                << " into subnet " << subnet
                                << "." << endl;
                }

                Credit * const c = Credit::New();
                c->vc.insert(f->vc);
                //std::cout << "vc: ";
                //for (auto vc: c->vc) {
                //    std::cout << vc << " ";
                //}
                //std::cout << "\n";
                INFO("Buffer state before WriteCredit - VC:%d available:%d\n",
                        *c->vc.begin(), _buf_states[n][subnet]->IsAvailableFor(*(c->vc.begin())));
                INFO("occupancy before:%d\n", _buf_states[n][subnet]->Occupancy());
                _net[subnet]->WriteCredit(c, n);
                INFO("Buffer state after WriteCredit - VC:%d available:%d\n",
                        *c->vc.begin(), _buf_states[n][subnet]->IsAvailableFor(*(c->vc.begin())));
                INFO("occupancy after:%d\n", _buf_states[n][subnet]->Occupancy());
                f->credit_returned = true;
                INFO("writing credit for fid:%d\n", f->pid);
    #ifdef TRACK_FLOWS
                ++_ejected_flits[f->cl][n];
    #endif
                _RetireFlit(f, n);
            }
        }
        flits[subnet].clear();
        _net[subnet]->Evaluate( );
        _net[subnet]->WriteOutputs( );
    }

    for (int i = 0; i < _nodes; ++i) {
        if (gReceiverBusyCycles[i] > 0) {
          --gReceiverBusyCycles[i];
        }
        if ((gReceiverBusyCycles[i] == 0) && !gReceiverBuffers[i].empty()) {
          auto id_cycles_pair = gReceiverBuffers[i].front();
          int flit_id = id_cycles_pair.first;
          int flit_processing_cycles = id_cycles_pair.second;
          gReceiverBusyCycles[i] = flit_processing_cycles;
          gReceiverBuffers[i].pop_front();
          INFO("fid:%d core:%d Rx finished get flit and set %d processing cycles "
              "(new buffer size:%zu)\n",
              flit_id, i, flit_processing_cycles, gReceiverBuffers[i].size());
          assert(flit_id >= 0);
          assert(flit_processing_cycles >= 0);
        }
      }

    ++_time;
    assert(_time);
    if(gTrace){
        cout<<"TIME "<<_time<<endl;
    }

}

void TrafficManagerSpike::_RetireFlit(Flit* f, int dest) {
    // Only retire the flit once the core is ready to receive
    //if (!_rx_processing_cycles_left[dest])
    //{
        // New code
    //_pe_rx_busy[dest] = false;
    INFO("Retiring flit fid:%d at time %d\n", f->id, _time);

    // ** This is the beginning of the old code **
    _deadlock_timer = 0;

    assert(_total_in_flight_flits[f->cl].count(f->id) > 0);
    _total_in_flight_flits[f->cl].erase(f->id);

    if(f->record) {
        assert(_measured_in_flight_flits[f->cl].count(f->id) > 0);
        _measured_in_flight_flits[f->cl].erase(f->id);
    }

    if ( f->watch ) {
        *gWatchOut << GetSimTime() << " | "
                << "node" << dest << " | "
                << "Retiring flit " << f->id
                << " (packet " << f->pid
                << ", src = " << f->src
                << ", dest = " << f->dest
                << ", hops = " << f->hops
                << ", flat = " << f->atime - f->itime
                << ")." << endl;
    }

    if ( f->head && ( f->dest != dest ) ) {
        ostringstream err;
        err << "Flit " << f->id << " arrived at incorrect output " << dest;
        Error( err.str( ) );
    }

    if((_slowest_flit[f->cl] < 0) ||
    (_flat_stats[f->cl]->Max() < (f->atime - f->itime)))
        _slowest_flit[f->cl] = f->id;
    _flat_stats[f->cl]->AddSample( f->atime - f->itime);
    if(_pair_stats){
        _pair_flat[f->cl][f->src*_nodes+dest]->AddSample( f->atime - f->itime );
    }

    if ( f->tail ) {
        Flit * head;
        if(f->head) {
            head = f;
        } else {
            map<int, Flit *>::iterator iter = _retired_packets[f->cl].find(f->pid);
            assert(iter != _retired_packets[f->cl].end());
            head = iter->second;
            _retired_packets[f->cl].erase(iter);
            assert(head->head);
            assert(f->pid == head->pid);
        }
        if ( f->watch ) {
            *gWatchOut << GetSimTime() << " | "
                    << "node" << dest << " | "
                    << "Retiring packet " << f->pid
                    << " (plat = " << f->atime - head->ctime
                    << ", nlat = " << f->atime - head->itime
                    << ", frag = " << (f->atime - head->atime) - (f->id - head->id) // NB: In the spirit of solving problems using ugly hacks, we compute the packet length by taking advantage of the fact that the IDs of flits within a packet are contiguous.
                    << ", src = " << head->src
                    << ", dest = " << head->dest
                    << ")." << endl;
        }

        //code the source of request, look carefully, its tricky ;)
        if (f->type == Flit::READ_REQUEST || f->type == Flit::WRITE_REQUEST) {
            PacketReplyInfo* rinfo = PacketReplyInfo::New();
            rinfo->source = f->src;
            rinfo->time = f->atime;
            rinfo->record = f->record;
            rinfo->type = f->type;
            _repliesPending[dest].push_back(rinfo);
        } else {
            if(f->type == Flit::READ_REPLY || f->type == Flit::WRITE_REPLY  ){
                _requestsOutstanding[dest]--;
            } else if(f->type == Flit::ANY_TYPE) {
                _requestsOutstanding[f->src]--;
            }

        }

        // Only record statistics once per packet (at tail)
        // and based on the simulation state
        if ( ( _sim_state == warming_up ) || f->record ) {

            _hop_stats[f->cl]->AddSample( f->hops );

            if((_slowest_packet[f->cl] < 0) ||
            (_plat_stats[f->cl]->Max() < (f->atime - head->itime)))
                _slowest_packet[f->cl] = f->pid;
            _plat_stats[f->cl]->AddSample( f->atime - head->ctime);
            _nlat_stats[f->cl]->AddSample( f->atime - head->itime);
            _frag_stats[f->cl]->AddSample( (f->atime - head->atime) - (f->id - head->id) );

            if(_pair_stats){
                _pair_plat[f->cl][f->src*_nodes+dest]->AddSample( f->atime - head->ctime );
                _pair_nlat[f->cl][f->src*_nodes+dest]->AddSample( f->atime - head->itime );
            }
        }

        if(f != head) {
            head->Free();
        }
    }
    if(f->head && !f->tail) {
        _retired_packets[f->cl].insert(make_pair(f->pid, f));
    } else {
        f->Free();
    }
}

int TrafficManagerSpike::_GeneratePacket( int source, int stype,
        int cl, int time, int dest, int processing_cycles, int subnet)
{
    assert(stype!=0);

    INFO("Generating packet at time:%d\n", time);
    Flit::FlitType packet_type = Flit::ANY_TYPE;
    int size = _GetNextPacketSize(cl); //input size
    int pid = _cur_pid++;
    assert(_cur_pid);
    //int packet_destination = _traffic_pattern[cl]->dest(source);
    int packet_destination = dest;
    bool record = false;
    bool watch = gWatchOut && (_packets_to_watch.count(pid) > 0);
    if(_use_read_write[cl]){
        if(stype > 0) {
            if (stype == 1) {
                packet_type = Flit::READ_REQUEST;
                size = _read_request_size[cl];
            } else if (stype == 2) {
                packet_type = Flit::WRITE_REQUEST;
                size = _write_request_size[cl];
            } else {
                ostringstream err;
                err << "Invalid packet type: " << packet_type;
                Error( err.str( ) );
            }
        } else {
            PacketReplyInfo* rinfo = _repliesPending[source].front();
            if (rinfo->type == Flit::READ_REQUEST) {//read reply
                size = _read_reply_size[cl];
                packet_type = Flit::READ_REPLY;
            } else if(rinfo->type == Flit::WRITE_REQUEST) {  //write reply
                size = _write_reply_size[cl];
                packet_type = Flit::WRITE_REPLY;
            } else {
                ostringstream err;
                err << "Invalid packet type: " << rinfo->type;
                Error( err.str( ) );
            }
            packet_destination = rinfo->source;
            time = rinfo->time;
            record = rinfo->record;
            _repliesPending[source].pop_front();
            rinfo->Free();
        }
    }

    if ((packet_destination <0) || (packet_destination >= _nodes)) {
        ostringstream err;
        err << "Incorrect packet destination " << packet_destination
            << " for stype " << packet_type;
        Error( err.str( ) );
    }

    if ( ( _sim_state == running ) ||
         ( ( _sim_state == draining ) && ( time < _drain_time ) ) ) {
        record = _measure_stats[cl];
    }

    //int subnetwork = ((packet_type == Flit::ANY_TYPE) ?
    //                  RandomInt(_subnets-1) :
    //                  _subnet[packet_type]);
    // jboyle change
    int subnetwork = subnet;

    if ( watch ) {
        *gWatchOut << GetSimTime() << " | "
                   << "node" << source << " | "
                   << "Enqueuing packet " << pid
                   << " at time " << time
                   << "." << endl;
    }

    for ( int i = 0; i < size; ++i ) {
        Flit * f  = Flit::New();
        f->id     = _cur_id++;
        assert(_cur_id);
        f->pid    = pid;
        f->watch  = watch | (gWatchOut && (_flits_to_watch.count(f->id) > 0));
        f->subnetwork = subnetwork;
        f->src    = source;
        f->ctime  = time;
        f->record = record;
        f->cl     = cl;
        f->processing_cycles = processing_cycles;

        _total_in_flight_flits[f->cl].insert(make_pair(f->id, f));
        if(record) {
            _measured_in_flight_flits[f->cl].insert(make_pair(f->id, f));
        }

        if(gTrace){
            cout<<"New Flit "<<f->src<<endl;
        }
        f->type = packet_type;

        if ( i == 0 ) { // Head flit
            f->head = true;
            //packets are only generated to nodes smaller or equal to limit
            f->dest = packet_destination;
        } else {
            f->head = false;
            f->dest = -1;
        }
        switch( _pri_type ) {
        case class_based:
            f->pri = _class_priority[cl];
            assert(f->pri >= 0);
            break;
        case age_based:
            f->pri = numeric_limits<int>::max() - time;
            assert(f->pri >= 0);
            break;
        case sequence_based:
            f->pri = numeric_limits<int>::max() - _packet_seq_no[source];
            assert(f->pri >= 0);
            break;
        default:
            f->pri = 0;
        }
        if ( i == ( size - 1 ) ) { // Tail flit
            f->tail = true;
        } else {
            f->tail = false;
        }

        f->vc  = -1;

        if ( f->watch ) {
            *gWatchOut << GetSimTime() << " | "
                       << "node" << source << " | "
                       << "Enqueuing flit " << f->id
                       << " (packet " << f->pid
                       << ") at time " << time
                       << "." << endl;
        }

        _partial_packets[source][cl].push_back( f );
    }

    return pid;
}

bool TrafficManagerSpike::_MessagesBeingReceived()
{
    for ( int subnet = 0; subnet < _subnets; ++subnet ) {
        for(int n = 0; n < _nodes; ++n) {
            const FlitChannel * inject = _net[subnet]->GetInject(n);
            const Router * router = inject->GetSink();
            assert(router);
            if (router->_rxBusy()) {
                return true;
            }
        }
    }
    return false;
}

bool TrafficManagerSpike::Run( )
{
    for ( int sim = 0; sim < _total_sims; ++sim ) {

        _time = 0;

        //remove any pending request from the previous simulations
        _requestsOutstanding.assign(_nodes, 0);
        for (int i=0;i<_nodes;i++) {
            while(!_repliesPending[i].empty()) {
                _repliesPending[i].front()->Free();
                _repliesPending[i].pop_front();
            }
        }

        //reset queuetime for all sources
        for ( int s = 0; s < _nodes; ++s ) {
            _qtime[s].assign(_classes, 0);
            _qdrained[s].assign(_classes, false);
        }

        // warm-up ...
        // reset stats, all packets after warmup_time marked
        // converge
        // draing, wait until all packets finish
        _sim_state    = warming_up;

        _ClearStats( );

        for(int c = 0; c < _classes; ++c) {
            _traffic_pattern[c]->reset();
            _injection_process[c]->reset();
        }

        if ( !_SingleSim( ) ) {
            cout << "Simulation failed, ending ..." << endl;
            return false;
        }

        // Empty any remaining packets
        cout << "Draining remaining packets ..." << endl;
        _empty_network = true;
        int empty_steps = 0;

        bool packets_left = false;
        for(int c = 0; c < _classes; ++c) {
            packets_left |= !_total_in_flight_flits[c].empty();
        }

        while( packets_left || _Pending() ) {
            _Step( );

            ++empty_steps;

            if ( empty_steps % 1000 == 0 ) {
                _DisplayRemaining( );
            }

            packets_left = false;
            for(int c = 0; c < _classes; ++c) {
                packets_left |= !_total_in_flight_flits[c].empty();
            }
        }
        //wait until all the credits are drained as well
        while(Credit::OutStanding()!=0){
            _Step();
        }

        // Wait until all message and neuron processing  has finished too
        cout << "Waiting for neurons/messages to be processed\n";
        while (_MessagesBeingReceived() || _NeuronProcessing()) {
            _Step();
        }
        _empty_network = false;

        //for the love of god don't ever say "Time taken" anywhere else
        //the power script depend on it
        cout << "Time taken is " << _time << " cycles" <<endl;

        if(_stats_out) {
            WriteStats(*_stats_out);
        }
        _UpdateOverallStats();
    }

    DisplayOverallStats();
    if(_print_csv_results) {
        DisplayOverallStatsCSV();
    }

    return true;
}


void TrafficManagerSpike::_ClearStats() {
    TrafficManager::_ClearStats();
    //_pe_rx_busy = std::vector<bool>(_nodes, false);
    //_rx_processing_cycles_left.clear();
    _tx_processing_cycles_left.clear();
    _message_count = std::vector<long long int>(_nodes, 0ULL);
    pending_events.clear();
}
