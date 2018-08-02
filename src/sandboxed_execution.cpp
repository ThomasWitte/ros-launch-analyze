#include <ros_launch_lint/sandboxed_execution.h>
#include <procxx/include/process.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <cstdlib>

NodeAnalyzer::NodeAnalyzer(AnalysisOptions opts) : opts {opts} {
    start_server();
}

NodeAnalyzer::~NodeAnalyzer() {
    stop_server();
}

void NodeAnalyzer::start_server() {
    fut = std::async(std::launch::async, [&]{
        // set up access channels to only log interesting things
        server.clear_access_channels(websocketpp::log::alevel::all);
        server.clear_error_channels(websocketpp::log::elevel::info);
        if (opts.debug_connection) {
            server.set_access_channels(websocketpp::log::alevel::access_core);
            server.set_access_channels(websocketpp::log::alevel::app);
            server.set_error_channels(websocketpp::log::elevel::info);
        }

        // Initialize the Asio transport policy
        server.init_asio();
        server.set_reuse_addr(true);

        // Bind the handlers we are using
        server.set_open_handler([&](const auto&) {
            server.get_alog().write(websocketpp::log::alevel::app, "client connected!");
        });
        server.set_close_handler([&](const auto&) {
            server.get_alog().write(websocketpp::log::alevel::app, "client disconnected!");
        });
        server.set_message_handler([&](const auto& /*connection_hdl*/, const auto& msg_ptr) {
            std::regex r {"<<([^>]+)>> ([^\\s]+) ([^\\s]+)"};
            std::smatch m;
            if (std::regex_match(msg_ptr->get_payload(), m, r)) {
                Port p;
                p.name = m[2];
                p.data_type = m[3];
                if (m[1] == "advertise")
                    p.type = Port::PUBLISHER;
                else if (m[1] == "subscribe")
                    p.type = Port::SUBSCRIBER;
                else if (m[1] == "advertiseService")
                    p.type = Port::SERVICE_ADVERTISE;
                else if (m[1] == "serviceClient")
                    p.type = Port::SERVICE_CLIENT;
                else
                    p.type = Port::NONE;
                received_ports.push_back(p);
            } else {
                ROS_WARN_STREAM("received invalid data from analyzed node: " << msg_ptr->get_payload());
            }
        });

        while (!exiting) {
            try {
                ROS_DEBUG("listening...");

                // Listen on port 34005
                server.listen(34005);

                // Start the server accept loop
                server.start_accept();

                // Start the ASIO io_service run loop
                server.run();
            } catch(...) {
                ROS_WARN("exception caught");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }

    });
}

void NodeAnalyzer::stop_server() {
    ROS_DEBUG("stopping server");
    exiting = true;
    server.stop_listening();
}

std::vector<Port> NodeAnalyzer::analyze_node(const NodeDesc& node, const std::vector<param_t>& global_params) {

    std::vector<std::string> arguments;
    if (opts.enable_sandbox) {
        arguments.push_back("firejail");
        arguments.push_back("--env=LD_LIBRARY_PATH=" + std::string(std::getenv("LD_LIBRARY_PATH")));
        arguments.push_back("--overlay-tmpfs");
        arguments.push_back("--quiet");
        arguments.push_back("--noprofile");
    }
    arguments.push_back(ros::package::getPath("ros_launch_lint") + "/get_topics.sh");

    // add global parameters
    for (const auto param : global_params) {
        arguments.push_back(param.first);
        arguments.push_back(param.second);
    }

    arguments.push_back("--");

    // package and node to start
    arguments.push_back(node.package);
    arguments.push_back(node.type);

    // split and add arguments
    if (!node.args.empty()) {
        std::stringstream ss {node.args};
        std::string s;
        while (ss >> s) {
            // don't split quoted strings
            if (s[0] == '\"') {
                for (std::string s2; ss >> s2 && s2[s2.size()-1] != '\"'; s = s + " " + s2)
                    ;
            }
            arguments.push_back(s);
        }
    }

    // add private parameters
    arguments.push_back("__name:=" + node.name);
    arguments.push_back("__ns:=" + node.path);
    for (const auto param : node.params) {
        arguments.push_back(std::string("_") + param.first + ":=" + param.second);
    }

    // create process
    procxx::process get_topics {arguments.front()};
    for (auto it = arguments.begin()+1; it != arguments.end(); it++)
        get_topics.add_argument(*it);

    if (opts.debug_cmdline) {
        std::stringstream ss;
        for (const auto& arg : arguments)
            ss << arg << " ";
        ROS_DEBUG_STREAM("cmd: " << ss.str());
    }

    // clear port data
    received_ports.clear();

    // execute analysis script
    get_topics.exec();

    if (opts.debug_output) {
        std::string line;
        while (std::getline(get_topics.error(), line))
            ROS_DEBUG_STREAM(line);
    }

    // wait for analysis script to finish
    get_topics.wait();

    return received_ports;
}

void sandboxed_execution(NodeTree& node_tree, bool debug = false) {
    AnalysisOptions opts {debug, debug, debug, true};
    NodeAnalyzer na {opts};

    for (auto it = node_tree.nodes.begin(); it != node_tree.nodes.end(); ++it) {
        // continue, if not a node
        if (it->type.empty())
            continue;

        ROS_INFO_STREAM("querying topics for " << it->name);

        auto ports = na.analyze_node(*it, node_tree.global_params);

        for (auto& p : ports)
            p.name = get_absolute_path(it.node, resolve_remaps(*it, p.name));

        it->ports = ports;

        ROS_INFO_STREAM("...found " << it->ports.size() << " topics/services");
        ROS_DEBUG_STREAM(*it);
    }
}
