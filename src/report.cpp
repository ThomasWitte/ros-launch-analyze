#include "ros_launch_lint/report.h"

void print_node_tree(const NodeTree& tree, std::ostream& output) {
    std::map<std::string, int> colors;
    int next_color = 31;
    for (auto it = tree.nodes.begin(); it != tree.nodes.end(); ++it) {
        for (int i = tree.nodes.depth(it); i > 0; --i) {
            output << "  ";
        }
        if (colors.find(it->launch_file) == colors.end()) {
            colors[it->launch_file] = next_color++;
        }

        std::string str = it->name;
        if (!it->type.empty()) {
            int padding_w = 32 - it->name.size() - 2*tree.nodes.depth(it);
            if (padding_w < 0) {
                str.resize(std::max(str.size() + padding_w - 3, 0ul));
                str += "...";
                padding_w = 0;
            }

            std::string padding(padding_w, ' ');
            str += padding + "[" + it->package + "/" + it->type + "]";
        }

        output << "\x1b[" << colors[it->launch_file] << "m"
                  << str
                  << "\x1b[0m" << std::endl;
    }

    output << std::endl;

    for (const auto& elt : colors) {
        output << "\x1b[" << elt.second << "m" << elt.first << "\x1b[0m" << std::endl;
    }
}

void print_topics(const NodeTree& tree, std::ostream& output) {
    //       topic                   publisher                 subscriber                type
    std::map<std::string, std::tuple<std::vector<std::string>, std::vector<std::string>, std::string>> topics;

    for (auto it = tree.nodes.begin(); it != tree.nodes.end(); ++it) {
        for (const auto& p : it->ports) {
            if (std::get<2>(topics[p.name]) != p.data_type && p.data_type != "*") {
                if (std::get<2>(topics[p.name]) != "" &&
                    std::get<2>(topics[p.name]) != "*") {

                    output << "Incompatible types at topic/service " << p.name << ": "
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
        output << elt.first << " (" << std::get<2>(elt.second) << "):" << std::endl;
        output << "  ";
        for (const auto& s : std::get<0>(elt.second))
            output << s << " ";
        output << "-> ";
        for (const auto& s : std::get<1>(elt.second))
            output << s << " ";
        output << std::endl;
    }
}

void print_dot(const NodeTree& tree, std::ostream& output) {
    output << "digraph {" << std::endl;

    int i = 0;
    for (auto it = tree.nodes.begin(); it != tree.nodes.end(); ++it) {
        if (it->type.empty())
            continue;

        ++i;

        std::string name = it->name;
        for (auto node = it.node->parent; node != nullptr; node = node->parent) {
            name = node->data.name + name;
        }

        output << "node" << i << "[label=\"" << name << "\"; shape=box];" << std::endl;

        for (const auto& p : it->ports) {
            switch (p.type) {
            case Port::PUBLISHER:
                output << "node" << i << " -> \"" << p.name << "\";" << std::endl;
                break;
            case Port::SUBSCRIBER:
                output << "\"" << p.name << "\" -> node" << i << ";" << std::endl;
                break;
            case Port::SERVICE_ADVERTISE:
                output << "node" << i << " -> {\"" << p.name << "\"[shape=octagon]};" << std::endl;
                break;
            case Port::SERVICE_CLIENT:
                output << "{\"" << p.name << "\"[shape=octagon]} -> node" << i << ";" << std::endl;
                break;
            default:
                break;
            }
        }
    }

    output << "}" << std::endl;
}