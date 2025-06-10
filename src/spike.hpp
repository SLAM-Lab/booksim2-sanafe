#ifndef _BOOKSIM_SPIKE_H_
#define _BOOKSIM_SPIKE_H_

struct SpikeEvent {
    enum class Type {
        SPIKE_PACKET,
        PROCESSING_ONLY
    };

    Type event_type;
    long int mid;
    int timestep;
    std::pair<std::string, int> src_neuron;
    std::pair<int, int> src_hw;
    std::pair<int, int> dest_hw;
    int hops{0};
    int spikes{0};
    double generation_delay;
    double network_latency;
    double processing_delay;
    double blocking_latency;

    // For spike packets only
    // Timestamps
    long long pending_cycle{-1LL};
    long long injection_cycle{-1LL};
    // The vector stores all intermediate hop timestamps i.e., the cycle at
    //  which the message is forwarded to the next router. Using this, we can
    //  figure out when the message is getting stuck
    std::vector<long long> hop_cycles;
    long long ejection_cycle{-1LL};

    // Delays
    long long blocked_cycles{0LL};
    long long network_cycles{0LL};

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
        event.generation_delay = processing_time;
        return event;
    }
};

#endif