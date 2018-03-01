#include <ros_launch_lint/sandboxed_execution.h>
#include <procxx/include/process.h>
#include <ros/ros.h>
#include <ros/package.h>

NodeAnalyzer::NodeAnalyzer() {
    start_server();
}

NodeAnalyzer::~NodeAnalyzer() {
    stop_server();
}

void NodeAnalyzer::start_server() {
    fut = std::async(std::launch::async, [&]{
        // set up access channels to only log interesting things
        server.clear_access_channels(websocketpp::log::alevel::all);
        //server.set_access_channels(websocketpp::log::alevel::access_core);
        //server.set_access_channels(websocketpp::log::alevel::app);

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

        // Listen on port 34005
        server.listen(34005);

        // Start the server accept loop
        server.start_accept();

        // Start the ASIO io_service run loop
        server.run();
    });
}

void NodeAnalyzer::stop_server() {
    server.stop_listening();
}

std::vector<Port> NodeAnalyzer::analyze_node(const NodeDesc& node, const std::vector<param_t>& global_params, AnalysisOptions opts) {

    // create process
    procxx::process get_topics {"firejail",
                                "--env=LD_LIBRARY_PATH=/home/thomas/catkin_ws/devel/lib:/home/thomas/ros_ws/devel/lib:/opt/ros/kinetic/lib:/home/thomas/local/lib",
                                "--overlay",
                                "--quiet",
                                ros::package::getPath("ros_launch_lint") + "/get_topics.sh"};

    if (opts.debug_cmdline)
        std::cout << "cmd: firejail --env=LD_LIBRARY_PATH=$LD_LIBRARY_PATH --overlay --quiet "
                  << ros::package::getPath("ros_launch_lint") << "/get_topics.sh";

    // add global parameters
    for (const auto param : global_params) {
        get_topics.add_argument(param.first);
        get_topics.add_argument(param.second);
        if (opts.debug_cmdline)
            std::cout << " " << param.first << " " << param.second;
    }

    get_topics.add_argument("--");
    if (opts.debug_cmdline)
        std::cout << " --";

    // package and node to start
    get_topics.add_argument(node.package);
    get_topics.add_argument(node.type);
    if (opts.debug_cmdline)
        std::cout << " " << node.package << " " << node.type;

    // split and add arguments (TODO: don't split quoted strings)
    if (!node.args.empty()) {
        std::stringstream ss {node.args};
        std::string s;
        while (ss >> s) {
            get_topics.add_argument(s);
            if (opts.debug_cmdline)
                std::cout << " " << s;
        }
    }

    // add private parameters
    for (const auto param : node.params) {
        get_topics.add_argument(std::string("_") + param.first + ":=" + param.second);
        if (opts.debug_cmdline)
            std::cout << " _" << param.first << ":=" << param.second;
    }

    if (opts.debug_cmdline)
        std::cout << std::endl;

    // clear port data
    received_ports.clear();

    // execute analysis script
    get_topics.exec();

    if (opts.debug_output) {
        std::string line;
        while (std::getline(get_topics.error(), line))
            std::cout << line << std::endl;
    }

    // wait for analysis script to finish
    get_topics.wait();

    return received_ports;
}
