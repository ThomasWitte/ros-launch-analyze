#include <string>
#include <cstdint>
#include <fstream>
#include <regex>
#include <iostream>

struct Port {
    std::string topic;
    std::string data_type;
    int64_t position;
    enum {NONE=0, PUBLISHER, SUBSCRIBER} type;
};

std::vector<Port> analyze_cpp_source(const std::string& source) {
    std::vector<Port> result;
    std::smatch sm;
//    std::regex r_pub {"\\.advertise<([^>]+)>\\(\\s*\"([^\"]+)\"\\s*,[^\\)]+\\)"};
//    std::regex r_sub {"\\.subscribe<([^>]+)>\\(\\s*\"([^\"]+)\"\\s*,[^\\)]+\\)"};
    std::regex r_pub {"\\.advertise<([^>]+)>\\(\\s*([^\\s,]+)\\s*,[^\\)]+\\)"};
    std::regex r_sub {"\\.subscribe<([^>]+)>\\(\\s*([^\\s,]+)\\s*,[^\\)]+\\)"};
    std::regex tf_listener {"(tf2_ros::|\\s)TransformListener(\\s+\\w+)?\\s*(\\([^\\)]+\\)|\\{[^\\}]+\\})"};
    std::regex tf_broadcaster {"(tf2_ros::|\\s)(Static)?TransformBroadcaster(\\s+\\w+)?\\s*(\\(\\)|\\{\\}|)"};
    std::string s = source;

    while (std::regex_search(s, sm, r_pub)) {
        result.emplace_back(Port {sm[2], sm[1], sm.position(), Port::PUBLISHER});
        s = sm.suffix().str();
    }

    s = source;
    while (std::regex_search(s, sm, r_sub)) {
        result.emplace_back(Port {sm[2], sm[1], sm.position(), Port::SUBSCRIBER});
        s = sm.suffix().str();
    }

    s = source;
    while (std::regex_search(s, sm, tf_listener)) {
        result.emplace_back(Port {"/tf", "geometry_msgs::TransformStamped", sm.position(), Port::SUBSCRIBER});
        result.emplace_back(Port {"/tf_static", "geometry_msgs::TransformStamped", sm.position(), Port::SUBSCRIBER});
        s = sm.suffix().str();
    }

    s = source;
    while (std::regex_search(s, sm, tf_broadcaster)) {
        if (sm[2] == "Static") {
            result.emplace_back(Port {"/tf_static", "geometry_msgs::TransformStamped", sm.position(), Port::PUBLISHER});
        } else {
            result.emplace_back(Port {"/tf", "geometry_msgs::TransformStamped", sm.position(), Port::PUBLISHER});
        }
        s = sm.suffix().str();
    }

    return result;
}

std::vector<Port> analyze_py_source(const std::string& source) {
    std::vector<Port> result;
    std::smatch sm;
    std::regex r_pub {"rospy\\.Publisher\\(\\s*([^\\s,]+)\\s*,\\s*([^\\s,]+)"};
    std::regex r_sub {"rospy\\.Subscriber\\(\\s*([^\\s,]+)\\s*,\\s*([^\\s,]+)"};
    std::regex tf_listener {"tf2_ros\\.TransformListener\\([^\\)]+\\)"};
    std::regex tf_broadcaster {"tf2_ros\\.(Static)?TransformBroadcaster\\(\\)"};
    std::string s = source;

    while (std::regex_search(s, sm, r_pub)) {
        result.emplace_back(Port {sm[1], sm[2], sm.position(), Port::PUBLISHER});
        s = sm.suffix().str();
    }

    s = source;
    while (std::regex_search(s, sm, r_sub)) {
        result.emplace_back(Port {sm[1], sm[2], sm.position(), Port::SUBSCRIBER});
        s = sm.suffix().str();
    }

    s = source;
    while (std::regex_search(s, sm, tf_listener)) {
        result.emplace_back(Port {"/tf", "geometry_msgs::TransformStamped", sm.position(), Port::SUBSCRIBER});
        result.emplace_back(Port {"/tf_static", "geometry_msgs::TransformStamped", sm.position(), Port::SUBSCRIBER});
        s = sm.suffix().str();
    }

    s = source;
    while (std::regex_search(s, sm, tf_broadcaster)) {
        if (sm[2] == "Static") {
            result.emplace_back(Port {"/tf_static", "geometry_msgs::TransformStamped", sm.position(), Port::PUBLISHER});
        } else {
            result.emplace_back(Port {"/tf", "geometry_msgs::TransformStamped", sm.position(), Port::PUBLISHER});
        }
        s = sm.suffix().str();
    }

    return result;
}

int main(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {

        std::ifstream file_in {argv[i], std::ios_base::in};

        if (!file_in.is_open()) {
            std::cerr << "could not open file " << argv[i] << std::endl;
            continue;
        }

        std::string source {std::istreambuf_iterator<char>{file_in},
                            std::istreambuf_iterator<char>{}};

        file_in.close();

        auto ports = analyze_cpp_source(source);
        auto tmp = analyze_py_source(source);
        ports.insert(ports.end(), tmp.begin(), tmp.end());

        std::cout << argv[i] << ":" << std::endl;
        for (const auto& p : ports) {
            std::cout << p.topic << " " << p.data_type << " " << p.position << " " << p.type << std::endl;
        }
        std::cout << std::endl;

    }
}
