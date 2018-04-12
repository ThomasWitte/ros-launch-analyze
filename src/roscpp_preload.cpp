#define _GNU_SOURCE 1

#include <dlfcn.h>
#include <ros/ros.h>
#include <ros/init.h>
#include <ros/topic_manager.h>
#include <actionlib/client/connection_monitor.h>
#include <iostream>
#include <future>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

// union that enables casting function pointers returned by dlsym to member pointers
template<typename memberT>
union u_ptm_cast {
    memberT pmember;
    struct vstruct {void *pvoid; char padding[8];} vs;

    static_assert(sizeof(memberT) == sizeof(vstruct), "void* and member* must have the same size");
};

// singleton that handles the websocket communication with the analysis plugin
struct AnalysisClient {
    using client_t = websocketpp::client<websocketpp::config::asio_client>;

    AnalysisClient() : connection_open(false) {
        start_client();
    }

    ~AnalysisClient() {
        client.close(hdl, websocketpp::close::status::normal, "shutdown client");
        asio_thread.join();
    }

    void start_client() {
        // set up access channels to only log interesting things
        client.clear_access_channels(websocketpp::log::alevel::all);
        //client.set_access_channels(websocketpp::log::alevel::connect);
        //client.set_access_channels(websocketpp::log::alevel::disconnect);
        //client.set_access_channels(websocketpp::log::alevel::app);

        // Initialize the Asio transport policy
        client.init_asio();

        // Bind the handlers we are using
        client.set_open_handler([&](websocketpp::connection_hdl){
            client.get_alog().write(websocketpp::log::alevel::app, "Connection opened!");
            connection_open = true;
        });
        client.set_close_handler([&](websocketpp::connection_hdl){
            client.get_alog().write(websocketpp::log::alevel::app, "Connection closed!");
        });
        client.set_fail_handler([&](websocketpp::connection_hdl){
            client.get_alog().write(websocketpp::log::alevel::app, "Connection failed!");
        });

        // Create a new connection to the given URI
        websocketpp::lib::error_code ec;
        client_t::connection_ptr con = client.get_connection("ws://localhost:34005", ec);
        if (ec) {
            client.get_alog().write(websocketpp::log::alevel::app,
                    "Get Connection Error: "+ec.message());
            return;
        }

        // Grab a handle for this connection so we can talk to it in a thread
        // safe manor after the event loop starts.
        hdl = con->get_handle();

        // Queue the connection. No DNS queries or network connections will be
        // made until the io_service event loop is run.
        client.connect(con);

        // Create a thread to run the ASIO io_service event loop
        asio_thread = websocketpp::lib::thread(&client_t::run, &client);
    }

    void log(std::string s) {
        while(!connection_open)
            std::this_thread::yield();
        client.send(hdl, s, websocketpp::frame::opcode::text);
    }

    client_t client;
    websocketpp::lib::thread asio_thread;
    websocketpp::connection_hdl hdl;
    std::atomic_bool connection_open;

    static std::unique_ptr<AnalysisClient> instance;
    static AnalysisClient* get_instance();
};

std::unique_ptr<AnalysisClient> AnalysisClient::instance;
AnalysisClient* AnalysisClient::get_instance() {
    if (!instance)
        instance = std::make_unique<AnalysisClient>();

    return instance.get();
}

namespace ros {

Publisher NodeHandle::advertise(AdvertiseOptions& opts) {
    typedef Publisher (NodeHandle::*advertise_t)(AdvertiseOptions&);

    static advertise_t orig_advertise = nullptr;

    if (!orig_advertise) {
        u_ptm_cast<advertise_t> tmp;
        tmp.pmember = nullptr;
        tmp.vs.pvoid = dlsym(RTLD_NEXT, "_ZN3ros10NodeHandle9advertiseERNS_16AdvertiseOptionsE");
        orig_advertise = tmp.pmember;
    }

    AnalysisClient::get_instance()->log("<<advertise>> " + names::resolve(opts.topic) + " " + opts.datatype);

    return (this->*orig_advertise)(opts);
}

Subscriber NodeHandle::subscribe(SubscribeOptions& opts) {
    typedef Subscriber (NodeHandle::*subscribe_t)(SubscribeOptions&);

    static subscribe_t orig_subscribe = nullptr;

    if (!orig_subscribe) {
        u_ptm_cast<subscribe_t> tmp;
        tmp.pmember = nullptr;
        tmp.vs.pvoid = dlsym(RTLD_NEXT, "_ZN3ros10NodeHandle9subscribeERNS_16SubscribeOptionsE");
        orig_subscribe = tmp.pmember;
    }

    AnalysisClient::get_instance()->log("<<subscribe>> " + names::resolve(opts.topic) + " " + opts.datatype);

    return (this->*orig_subscribe)(opts);
}

ServiceServer NodeHandle::advertiseService(AdvertiseServiceOptions &opts) {
    typedef ServiceServer (NodeHandle::*advertise_service_t)(AdvertiseServiceOptions&);

    static advertise_service_t orig_advertise = nullptr;

    if (!orig_advertise) {
        u_ptm_cast<advertise_service_t> tmp;
        tmp.pmember = nullptr;
        tmp.vs.pvoid = dlsym(RTLD_NEXT, "_ZN3ros10NodeHandle16advertiseServiceERNS_23AdvertiseServiceOptionsE");
        orig_advertise = tmp.pmember;
    }

    AnalysisClient::get_instance()->log("<<advertiseService>> " + names::resolve(opts.service) + " " + opts.datatype);

    return (this->*orig_advertise)(opts);
}

ServiceClient NodeHandle::serviceClient(ServiceClientOptions &opts) {
    typedef ServiceClient (NodeHandle::*service_client_t)(ServiceClientOptions&);

    static service_client_t orig_client = nullptr;

    if (!orig_client) {
        u_ptm_cast<service_client_t> tmp;
        tmp.pmember = nullptr;
        tmp.vs.pvoid = dlsym(RTLD_NEXT, "_ZN3ros10NodeHandle13serviceClientERNS_20ServiceClientOptionsE");
        orig_client = tmp.pmember;
    }

    AnalysisClient::get_instance()->log("<<serviceClient>> " + names::resolve(opts.service) + " " + opts.md5sum);

    return (this->*orig_client)(opts);
}

bool ServiceClient::waitForExistence(Duration) {
    return true;
}

void start() {
    typedef void (*func_t)(void);

    static func_t orig_start = nullptr;
    if (!orig_start)
        orig_start = reinterpret_cast<func_t>(dlsym(RTLD_NEXT, "_ZN3ros5startEv"));

    orig_start();

    bool sim_time = false;
    param::param("/use_sim_time", sim_time, sim_time);

    if (sim_time)
        AnalysisClient::get_instance()->log("<<subscribe>> " + names::resolve("/clock") + " rosgraph_msgs/Clock");

    // if (!(g_init_options & init_options::NoRosout)) {
        AnalysisClient::get_instance()->log("<<advertise>> " + names::resolve("/rosout") + " rosgraph_msgs/Log");
    // }

    AnalysisClient::get_instance()->log("<<advertiseService>> " + names::resolve("~get_loggers") + " roscpp/GetLoggers");
    AnalysisClient::get_instance()->log("<<advertiseService>> " + names::resolve("~set_logger_level") + " roscpp/SetLoggerLevel");
}

namespace service {

bool waitForService(const std::string&, ros::Duration) {
    return true;
}

bool waitForService(const std::string&, int32_t) {
    return true;
}

} // ns service
} // ns ros

namespace actionlib {

bool ConnectionMonitor::waitForActionServerToStart(const ros::Duration&, const ros::NodeHandle&) {
    return true;
}

bool ConnectionMonitor::isServerConnected() {
    return true;
}

}
