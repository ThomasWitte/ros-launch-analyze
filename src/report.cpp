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

void print_dot(const NodeListVisitor::tree_t& nodes) {
    std::cout << "digraph {" << std::endl;

    int i = 0;
    for (auto it = nodes.begin(); it != nodes.end(); ++it, ++i) {
        if (it->type.empty())
            continue;

        std::string name = it->name;
        for (auto node = it.node->parent; node != nullptr; node = node->parent) {
            name = node->data.name + name;
        }

        std::cout << "node" << i << "[label=\"" << name << "\"; shape=box];" << std::endl;

        for (const auto& p : it->ports) {
            switch (p.type) {
            case Port::PUBLISHER:
                std::cout << "node" << i << " -> \"" << p.name << "\";" << std::endl;
                break;
            case Port::SUBSCRIBER:
                std::cout << "\"" << p.name << "\" -> node" << i << ";" << std::endl;
                break;
            case Port::SERVICE_ADVERTISE:
                std::cout << "node" << i << " -> {\"" << p.name << "\"[shape=octagon]};" << std::endl;
                break;
            case Port::SERVICE_CLIENT:
                std::cout << "{\"" << p.name << "\"[shape=octagon]} -> node" << i << ";" << std::endl;
                break;
            default:
                break;
            }
        }
    }

    std::cout << "}" << std::endl;
}
