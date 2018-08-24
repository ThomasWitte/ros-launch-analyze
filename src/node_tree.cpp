#include "ros_launch_lint/node_tree.h"

NodeListVisitor::NodeListVisitor(const std::string& root_xml) {
    nss.push(Namespace {"/"});
    file_stack.push(root_xml);
    tree.nodes.insert(tree.nodes.begin(), NodeDesc {"/", "", "", "", root_xml});
    it = create_path(nss.top().name);
}

bool NodeListVisitor::VisitExit (const XMLElement &elt) {
    if (std::string(elt.Name()) == "include") {
        file_stack.pop();
    }

    if (std::string(elt.Name()) == "node") {

        // save Namespace and all remaps for this node
        auto current_nss = nss;
        Namespace result_ns = current_nss.top();
        current_nss.pop();

        for (; !current_nss.empty(); current_nss.pop()) {
            for (const auto& remap : current_nss.top().remaps)
                result_ns.remaps.push_back(remap);
        }

        // remove node from namespace stack
        nss.pop();
        it = create_path(nss.top().name);

        // create child node in NodeTree
        tree.nodes.append_child(it, NodeDesc {elt.Attribute("name"),
                                              get_absolute_path(it.node, it->name),
                                              elt.Attribute("type"),
                                              elt.Attribute("pkg"),
                                              file_stack.top(),
                                              result_ns,
                                              private_params,
                                              elt.Attribute("args") ? elt.Attribute("args") : "",
                                              std::vector<Port>(),
                                              elt});
        // reset private parameters
        private_params.clear();
    } else if ((std::string(elt.Name()) == "param" || std::string(elt.Name()) == "rosparam")
            && elt.Parent() && std::string(elt.Parent()->ToElement()->Name()) != "node") {

        // we are not inside a node -> the param is global
        for (auto& p : private_params) {
            if (p.first[0] != '/')
                p.first = get_absolute_path(it.node, it->name + p.first);
            tree.global_params.push_back(p);
        }
        private_params.clear();
    }

    if (std::string(elt.Name()) == "group" ||
        std::string(elt.Name()) == "test" ||
        std::string(elt.Name()) == "include" ||
        std::string(elt.Name()) == "rosparam") {

        nss.pop();
        it = create_path(nss.top().name);
    }

    return true;
}

void NodeListVisitor::handle_rosparam(const XMLElement& elt, std::string param_prefix) {
    //TODO: params can be remapped!

    std::string command = "load";
    if (auto att = elt.Attribute("command"); att != nullptr) {
        command = att;
    }

    std::string param = "";
    if (auto att = elt.Attribute("param"); att != nullptr) {
        param = param_prefix + att;
    }

    if (command == "load") {
        YAML::Node yaml;

        std::string file = elt.Attribute("file") ? elt.Attribute("file") : "";

        //load file or text
        if (!file.empty()) {
            yaml = YAML::LoadFile(file);
        } else {
            file = "[inline yaml]";
            yaml = YAML::Load(elt.FirstChild()->ToText()->Value());
        }

        ROS_DEBUG_STREAM("loaded rosparam " << file << " -> " << yaml);

        switch (yaml.Type()) {
        case YAML::NodeType::Map:
            for (const auto& p : yaml) {
                auto k = p.first.as<std::string>();
                std::stringstream ss;
                ss << p.second;
                auto v = ss.str();
                // replace param if it exists
                if (auto pos = std::find_if(private_params.begin(), private_params.end(), [&](const auto& p){return k == p.first;}); pos != private_params.end()) {
                    *pos = std::make_pair(k,v);
                    ROS_DEBUG_STREAM("replace parameter: " << k << " : " << v);
                } else {
                    private_params.push_back(std::make_pair(k, v));
                    ROS_DEBUG_STREAM("set parameter: " << k << " : " << v);
                }
            }
            break;

        case YAML::NodeType::Scalar:
        case YAML::NodeType::Sequence:
        {
                std::stringstream ss;
                ss << yaml;

                if (auto pos = std::find_if(private_params.begin(), private_params.end(), [&](const auto& p){return param == p.first;}); pos != private_params.end()) {
                    *pos = std::make_pair(param, ss.str());
                    ROS_DEBUG_STREAM("replace parameter: " << param << " : " << yaml);
                } else {
                    private_params.push_back(std::make_pair(param, ss.str()));
                    ROS_DEBUG_STREAM("set parameter: " << param << " : " << yaml);
                }
            break;
        }

        default:
            ROS_WARN_STREAM("Problem loading paramter file " << file << "! " << yaml);
        }
    } else if (command == "delete") {
        // ignored, as I don't understand how this is supposed to work
    } else if (command == "dump") {
        // ignored
    }
}

bool NodeListVisitor::VisitEnter (const XMLElement &elt, const XMLAttribute *) {
    if (std::string(elt.Name()) == "param") {
        if (elt.Attribute("name") && elt.Attribute("value")) {
            private_params.push_back(std::make_pair(elt.Attribute("name"),
                                                    elt.Attribute("value")));
        }
    }

    if (std::string(elt.Name()) == "rosparam") {
        handle_rosparam(elt);
    }

    if (std::string(elt.Name()) == "remap") {
        nss.top().remaps.emplace_back(std::make_pair(elt.Attribute("from"),
                                                     elt.Attribute("to")));
    }

    if (std::string(elt.Name()) == "group" ||
        std::string(elt.Name()) == "node" ||
        std::string(elt.Name()) == "test" ||
        std::string(elt.Name()) == "rosparam" ||
        std::string(elt.Name()) == "include") {

        std::string ns = nss.top().name;

        if (elt.Attribute("ns")) {
            ns = elt.Attribute("ns");
        }

        if (ns[0] != '/') {
            ns = nss.top().name + ns;
        }

        if (ns[ns.size()-1] != '/') {
            ns = ns + '/';
        }

        nss.push(Namespace {ns});

        if (elt.Attribute("ns")) {
            it = create_path(ns);
        }
    }

    if (std::string(elt.Name()) == "include") {
        file_stack.push(elt.Attribute("file"));
    }
    return true;
}

NodeTree::tree_t::iterator NodeListVisitor::create_path(const std::string& path) {
    NodeTree::tree_t::pre_order_iterator tree_it = tree.nodes.begin();

    size_t pos = 0;
    while (pos != std::string::npos) {
        auto new_pos = path.find('/', pos+1);
        std::string ns = path.substr(pos+1, new_pos-pos);
        pos = new_pos;
        if (ns.empty())
            continue;

        unsigned num_children = tree_it.number_of_children();
        if (num_children == 0) {
            tree_it = tree.nodes.append_child(tree_it,
                                              NodeDesc {ns, get_absolute_path(tree_it.node, tree_it->name), "", "", file_stack.top()});
        } else {
            bool found = false;
            for (auto child_it = tree_it.begin(); child_it != tree_it.end(); ++child_it) {
                if (child_it->name == ns) {
                    tree_it = child_it;
                    found = true;
                    break;
                }
            }
            if (!found) {
                tree_it = tree.nodes.append_child(tree_it,
                                                  NodeDesc {ns, get_absolute_path(tree_it.node, tree_it->name), "", "", file_stack.top()});
            }
        }
    }

    return tree_it;
}

std::string resolve_remaps(const NodeDesc& node, std::string path) {

    ROS_DEBUG_STREAM("Resolving remaps for " << path);

    for (const auto& p : node.ns.remaps) {
        if (path == p.first || path == (node.ns.name + p.first)) {
            path = p.second;
            ROS_DEBUG_STREAM("...remapped to " << path);
            return path;
        } else {
            ROS_DEBUG_STREAM("remap " << p.first << " -> " << p.second << " does not match");
        }
    }

    ROS_DEBUG_STREAM("...no remap found");

    return path;
}

std::ostream& operator<< (std::ostream& out, const NodeDesc& desc) {
    out << desc.path << desc.name << std::endl;

    for(const auto& p : desc.params)
        out << "  " << p.first << "=" << p.second << std::endl;

    out << std::endl;

    for(const auto& p : desc.ports) {
        out << "  " << p.name << " [" << p.data_type << "] class:";

        switch(p.type) {
        case Port::NONE:
            out << "none";
            break;
        case Port::PUBLISHER:
            out << "pub";
            break;
        case Port::SUBSCRIBER:
            out << "sub";
            break;
        case Port::SERVICE_ADVERTISE:
            out << "adv";
            break;
        case Port::SERVICE_CLIENT:
            out << "call";
            break;
        }

        out << std::endl;
    }

    return out;
}
