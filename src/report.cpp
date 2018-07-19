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

void output_remaps(const NodeDesc& desc, XMLElement* node) {
    for (const auto& remap : desc.ns.remaps) {
        auto *remap_elt = node->GetDocument()->NewElement("remap");
        remap_elt->SetAttribute("from", remap.first.c_str());
        remap_elt->SetAttribute("to", remap.second.c_str());
        node->InsertEndChild(remap_elt);
    }
}

void output_private_params(const NodeDesc& desc, XMLElement* node) {
    for (const auto& param : desc.params) {
        auto *param_elt = node->GetDocument()->NewElement("param");
        param_elt->SetAttribute("name", param.first.c_str());
        param_elt->SetAttribute("value", param.second.c_str());
        node->InsertEndChild(param_elt);
    }
}

void output_global_params(const NodeTree& tree, XMLElement* node) {
    for (const auto& param : tree.global_params) {
        auto *param_elt = node->GetDocument()->NewElement("param");
        param_elt->SetAttribute("name", param.first.c_str());
        param_elt->SetAttribute("value", param.second.c_str());
        node->InsertEndChild(param_elt);
    }
}

void output_port_annotations(const NodeDesc& desc, XMLElement* node) {
    XMLDocument comment_doc;
    auto *topic_element = comment_doc.NewElement("topics");
    auto *service_element = comment_doc.NewElement("services");

    for (const Port& port : desc.ports) {
        auto *port_elt = comment_doc.NewElement("topic");
        port_elt->SetAttribute("name", port.name.c_str());
        port_elt->SetAttribute("type", port.data_type.c_str());

        switch (port.type) {
        case Port::PUBLISHER:
            port_elt->SetAttribute("class", "pub");
            topic_element->InsertEndChild(port_elt);
            break;
        case Port::SUBSCRIBER:
            port_elt->SetAttribute("class", "sub");
            topic_element->InsertEndChild(port_elt);
            break;
        case Port::SERVICE_ADVERTISE:
            port_elt->SetAttribute("class", "adv");
            service_element->InsertEndChild(port_elt);
            break;
        case Port::SERVICE_CLIENT:
            port_elt->SetAttribute("class", "call");
            service_element->InsertEndChild(port_elt);
            break;
        default:
            break;
        }
    }

    comment_doc.InsertEndChild(topic_element);
    comment_doc.InsertEndChild(service_element);

    XMLPrinter comment_printer;
    comment_doc.Print(&comment_printer);
    node->InsertEndChild(node->GetDocument()->NewComment(comment_printer.CStr()));
}

void print_xml(const NodeTree& tree, std::ostream& output) {
    XMLDocument doc;
    XMLPrinter printer;

    doc.InsertEndChild(doc.NewDeclaration("xml version=\"1.0\""));
    auto* root = doc.NewElement("launch");
    doc.InsertEndChild(root);

    for (const NodeDesc& desc : tree.nodes) {
        // skip groups etc
        if (desc.type.empty())
            continue;

        auto *node = doc.NewElement("node");

        node->SetAttribute("name", desc.name.c_str());
        node->SetAttribute("ns", desc.path.c_str());
        node->SetAttribute("pkg", desc.package.c_str());
        node->SetAttribute("type", desc.type.c_str());

        output_remaps(desc, node);
        output_private_params(desc, node);
        output_port_annotations(desc, node);

        root->InsertEndChild(node);
    }

    output_global_params(tree, root);

    doc.Print(&printer);
    output << printer.CStr();
}
