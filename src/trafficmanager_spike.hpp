// trafficmanager_spike.hpp
#ifndef _TRAFFICMANAGER_SPIKE_HPP_
#define _TRAFFICMANAGER_SPIKE_HPP_

#include <fstream>
#include <queue>
#include <map>
#include "trafficmanager.hpp"

#define CORES_PER_TILE 4

// Custom debug print routine with file and line number..
#ifdef TRACE
#define INFO(...) do { \
    fprintf(stdout, "[%s:%d:%s()] ", __FILE__, __LINE__, __func__); \
    fprintf(stdout, __VA_ARGS__); \
} while (0)
#else
#define INFO(...) do {} while (0)
#endif

struct SpikeEvent {
    enum class Type {
        SPIKE_PACKET,
        PROCESSING_ONLY
    };

    Type event_type;
    int timestep;
    std::pair<int, int> src_neuron;
    std::pair<int, int> src_hw;
    std::pair<int, int> dest_hw;
    int hops;
    int spikes;
    double generation_latency;
    double network_latency;
    double processing_latency;
    double blocking_latency;

    // For spike packets only
    long long injection_cycle;
    long long ejection_cycle;

    // Constructor for processing-only events
    static SpikeEvent CreateProcessingEvent(
        int timestep,
        std::pair<int, int> src_neuron,
        std::pair<int, int> src_hw,
        double processing_time) {
        SpikeEvent event;
        event.event_type = Type::PROCESSING_ONLY;
        event.timestep = timestep;
        event.src_neuron = src_neuron;
        event.src_hw = src_hw;
        event.dest_hw = std::make_pair(-1, -1);  // Invalid destination
        event.generation_latency = processing_time;
        return event;
    }
};

class TrafficManagerSpike : public TrafficManager {
protected:
    std::string trace_file;
    std::ifstream trace_stream;
    std::map<int, std::queue<SpikeEvent>> pending_events;
    // TODO: implement core queues for input and output spikes
    //std::map<int, std::queue<SpikeEvent>> packets_out; // To send
    //std::map<int, std::queue<SpikeEvent>> packets_in; // To receive
    //std::map<int, long long> core_busy_until;
    std::vector<bool> _pe_rx_busy{};
    std::vector<bool> _pe_tx_busy{};
    std::vector<std::queue<int>> _received_flits{};
    std::map<int, long long int> _rx_processing_cycles_left;
    std::map<int, long long int> _tx_processing_cycles_left;
    std::map<int, long long int> _flit_processing_cycles;

    double clock_period;

    virtual void _RetireFlit(Flit* flit, int dest);
    //virtual bool _ProcessWaiting(int input, int output);
    //virtual void _GeneratePacket(int source, int size, int cl, int time);
    virtual void _Inject();

public:
    TrafficManagerSpike(const Configuration& config, const vector<Network*>& net);
    virtual ~TrafficManagerSpike() override;
    static TrafficManagerSpike * New(Configuration const & config,
			      vector<Network *> const & net);

    void ReadNextTrace();
    int CyclesFromTime(double time_seconds);
    bool _Pending() override;
    bool Run( ) override;


protected:
    bool _SingleSim() override;
    void _ClearStats() override;
    void _Step() override;
    bool _MessagesBeingReceived();


private:
    bool OpenTrace();
    bool _MessageProcessingFinished();
    bool _InjectionPossible(const int source, const int dest, const int subnet);
    int _GeneratePacket(int source, int stype, int c1, int time, int dest, int processing_cycles, int subnet);
};

#endif
