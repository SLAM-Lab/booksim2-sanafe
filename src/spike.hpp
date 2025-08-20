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
    int subnet;
    std::pair<std::string, int> src_neuron;
    std::pair<int, int> src_hw;
    std::pair<int, int> dest_hw;
    int hops{0};
    int spikes{0};
    int buffered_along_path{0};
    int max_buffered_along_path{0};
    int buffered_squared{0};
    int buffered_and_adjacent_along_path{0};

    double generation_delay;
    double network_latency;
    double processing_delay;
    double blocking_latency;
    double mean_processing_delay; // Globally across the network
    double var_processing_delay; // Globally across the network

    int buffered_dest{0};
    int full_dest_tile{0};
    double mean_processing_dest;
    double var_processing_dest;

    double mean_processing_flow; // For all in flight messages sharing the flow
    double var_processing_flow; // For all in flight messages sharing the flow
    double mean_processing_path; // For all in flight messages sharing any link in the path
    double var_processing_path; // For all in flight messages sharing any link in the path

    int sharing_along_flow{0}; // Total messages sharing the flow/entire path
    int sharing_along_path{0}; // Total messages sharing any link in the path
    int max_sharing_along_path{0}; // Maximum messages sharing any given link in the path
    int sharing_path_squared{0}; // Squared messages sharing a link, per-link

    // For spike packets only
    // Timestamps
    long long tx_start_cycle{-1LL};
    long long tx_ready_cycle{-1LL};
    long long injection_cycle{-1LL};
    // The vector stores all intermediate hop timestamps i.e., the cycle at
    //  which the message is forwarded to the next router. Using this, we can
    //  figure out when the message is getting stuck
    std::vector<long long> hop_cycles;
    long long ejection_cycle{-1LL};
    long long processing_cycle{-1LL};
    long long processing_finished_cycle{-1LL};

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