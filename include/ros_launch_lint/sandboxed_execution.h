#ifndef SANDBOXED_EXECUTION_H
#define SANDBOXED_EXECUTION_H

#include <string>
#include <utility>
#include <vector>
#include <future>
#include <regex>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

using param_t = std::pair<std::string, std::string>;

struct Port {
    std::string name;
    std::string data_type;
    int64_t position;
    enum {NONE=0, PUBLISHER, SUBSCRIBER, SERVICE_ADVERTISE, SERVICE_CLIENT} type;
};

struct NodeDesc {
    std::string name;
    std::string type;
    std::string package;
    std::string launch_file;

    // params
    std::vector<param_t> params;
    std::string args;

    // topics
    std::vector<Port> ports;

    friend std::ostream& operator<< (std::ostream& out, const NodeDesc& desc);
};

struct AnalysisOptions {
    bool debug_cmdline = false;
    bool debug_output = true;
};

class NodeAnalyzer {
    using server_t = websocketpp::server<websocketpp::config::asio>;

public:
    NodeAnalyzer();
    ~NodeAnalyzer();

    std::vector<Port> analyze_node(const NodeDesc& node,
                                   const std::vector<param_t>& global_params,
                                   AnalysisOptions opts = AnalysisOptions());

private:
    void start_server();
    void stop_server();

    std::vector<Port> received_ports;

    server_t server;
    std::future<void> fut;
    bool exiting = false;
};

std::ostream& operator<< (std::ostream& out, const NodeDesc& desc);

#endif // SANDBOXED_EXECUTION_H
