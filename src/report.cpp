#include "ros_launch_lint/report.h"

void print_node_tree(const NodeListVisitor::tree_t& nodes) {
    std::map<std::string, int> colors;
    int next_color = 31;
    for (auto it = nodes.begin(); it != nodes.end(); ++it) {
        for (int i = nodes.depth(it); i > 0; --i) {
            std::cout << "  ";
        }
        if (colors.find(it->launch_file) == colors.end()) {
            colors[it->launch_file] = next_color++;
        }

        std::string str = it->name;
        if (!it->type.empty()) {
            int padding_w = 32 - it->name.size() - 2*nodes.depth(it);
            if (padding_w < 0) {
                str.resize(std::max(str.size() + padding_w - 3, 0ul));
                str += "...";
                padding_w = 0;
            }

            std::string padding(padding_w, ' ');
            str += padding + "[" + it->package + "/" + it->type + "]";
        }

        std::cout << "\x1b[" << colors[it->launch_file] << "m"
                  << str
                  << "\x1b[0m" << std::endl;
    }

    std::cout << std::endl;

    for (const auto& elt : colors) {
        std::cout << "\x1b[" << elt.second << "m" << elt.first << "\x1b[0m" << std::endl;
    }
}

void print_topics(const NodeListVisitor::tree_t& nodes) {
    //       topic                   publisher                 subscriber                type
    std::map<std::string, std::tuple<std::vector<std::string>, std::vector<std::string>, std::string>> topics;

    for (auto it = nodes.begin(); it != nodes.end(); ++it) {
        for (const auto& p : it->ports) {
            if (std::get<2>(topics[p.name]) != p.data_type && p.data_type != "*") {
                if (std::get<2>(topics[p.name]) != "" &&
                    std::get<2>(topics[p.name]) != "*") {

                    std::cout << "Incompatible types at topic/service " << p.name << ": "
                              << std::get<2>(topics[p.name]) << "!=" << p.data_type << std::endl;
                } else {
                    std::get<2>(topics[p.name]) = p.data_type;
                }
            }

            if (p.type == Port::PUBLISHER || p.type == Port::SERVICE_ADVERTISE)
                std::get<0>(topics[p.name]).push_back(it->name);
            if (p.type == Port::SUBSCRIBER || p.type == Port::SERVICE_CLIENT)
                std::get<1>(topics[p.name]).push_back(it->name);
        }
    }

    for (const auto& elt : topics) {
        std::cout << elt.first << " (" << std::get<2>(elt.second) << "):" << std::endl;
        std::cout << "  ";
        for (const auto& s : std::get<0>(elt.second))
            std::cout << s << " ";
        std::cout << "-> ";
        for (const auto& s : std::get<1>(elt.second))
            std::cout << s << " ";
        std::cout << std::endl;
    }
}

void print_launch_file_info(const NodeListVisitor::tree_t& node_tree) {
    print_node_tree(node_tree);
    print_topics(node_tree);
}
