#ifndef _BOOKSIM_SPIKE_H_
#define _BOOKSIM_SPIKE_H_

struct SpikeEvent {
    enum class Type {
        SPIKE_PACKET,
        PROCESSING_ONLY
    };

    Type event_type;
    int timestep;
    std::pair<std::string, int> src_neuron;
    std::pair<int, int> src_hw;
    std::pair<int, int> dest_hw;
    int hops{0};
    int spikes{0};
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
            std::pair<std::string, int> src_neuron,
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

#endif