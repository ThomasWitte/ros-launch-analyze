#ifndef SANDBOXED_EXECUTION_H
#define SANDBOXED_EXECUTION_H

#include <string>
#include <utility>
#include <vector>
#include <future>
#include <regex>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <ros_launch_lint/node_tree.h>

struct AnalysisOptions {
    bool debug_cmdline = false;
    bool debug_output = false;
    bool debug_connection = false;

    bool enable_sandbox = true;
};

class NodeAnalyzer {
    using server_t = websocketpp::server<websocketpp::config::asio>;

public:
    NodeAnalyzer(AnalysisOptions opts = AnalysisOptions());
    ~NodeAnalyzer();

    std::vector<Port> analyze_node(const NodeDesc& node,
                                   const std::vector<param_t>& global_params);

private:
    void start_server();
    void stop_server();

    AnalysisOptions opts;

    std::vector<Port> received_ports;

    server_t server;
    std::future<void> fut;
    bool exiting = false;
};

void sandboxed_execution(NodeTree& node_tree, bool debug);

#endif // SANDBOXED_EXECUTION_H
