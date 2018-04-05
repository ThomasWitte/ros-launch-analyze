#include "ros_launch_lint/node_tree.h"

NodeListVisitor::NodeListVisitor(const std::string& root_xml) {
    nss.push("/");
    file_stack.push(root_xml);
    nodes.insert(nodes.begin(), NodeDesc {"/", "", "", root_xml, std::vector<param_t>(), "", std::vector<Port>()});
    it = create_path(nss.top());
}

bool NodeListVisitor::Visit (const XMLComment &elt) {
    // assert that parent is a node tag
    if (elt.Parent() && elt.Parent()->ToElement() &&
        std::string(elt.Parent()->ToElement()->Name()) == "node") {

        // try to parse the comment as xml
        std::string comment = elt.Value();
        XMLDocument comment_doc;
        if (comment_doc.Parse(comment.c_str(), comment.size()) == XML_NO_ERROR) {
            const XMLNode* topic_node = comment_doc.FirstChild();
            if (!topic_node || !topic_node->ToElement() || std::string(topic_node->ToElement()->Name()) != "topics" || std::string(topic_node->ToElement()->Name()) != "services") {
                return true;
            }

            for (const XMLNode* node = topic_node->FirstChild(); node != nullptr; node = node->NextSibling()) {
                const XMLElement* elt = node->ToElement();

                if (elt) {
                    std::string cls = elt->Attribute("class");
                    Port p {elt->Attribute("name"),
                            elt->Attribute("type"),
                            -1,
                            (cls == "pub" ? Port::PUBLISHER
                          : (cls == "sub" ? Port::SUBSCRIBER
                          : (cls == "adv" ? Port::SERVICE_ADVERTISE
                          : (cls == "call" ? Port::SERVICE_CLIENT
                          : Port::NONE))))};
                    node_ports.push_back(p);
                }
            }
        }
    }

    return true;
}

bool NodeListVisitor::VisitExit (const XMLElement &elt) {
    if (std::string(elt.Name()) == "group" ||
        std::string(elt.Name()) == "node" ||
        std::string(elt.Name()) == "test" ||
        std::string(elt.Name()) == "include") {
        if (elt.Attribute("ns")) {
            nss.pop();
            it = create_path(nss.top());
        }
    }

    if (std::string(elt.Name()) == "include") {
        file_stack.pop();
    }

    if (std::string(elt.Name()) == "node") {
        nodes.append_child(it, NodeDesc {elt.Attribute("name"),
                                         elt.Attribute("type"),
                                         elt.Attribute("pkg"),
                                         file_stack.top(),
                                         private_params,
                                         elt.Attribute("args") ? elt.Attribute("args") : "",
                                         node_ports});
        private_params.clear();
        node_ports.clear();
    } else if (std::string(elt.Name()) == "param"
            && elt.Parent() && std::string(elt.Parent()->ToElement()->Name()) != "node") {

        // we are not inside a node -> the param is global
        for (const auto& p : private_params)
            global_params.push_back(p);
        private_params.clear();
    }

    return true;
}

bool NodeListVisitor::VisitEnter (const XMLElement &elt, const XMLAttribute *) {
    if (std::string(elt.Name()) == "param") {
        if (elt.Attribute("name") && elt.Attribute("value")) {
            private_params.push_back(std::make_pair(elt.Attribute("name"),
                                                    elt.Attribute("value")));
        }
    }

    if (std::string(elt.Name()) == "group" ||
        std::string(elt.Name()) == "node" ||
        std::string(elt.Name()) == "test" ||
        std::string(elt.Name()) == "include") {

        if (elt.Attribute("ns")) {
            std::string ns = elt.Attribute("ns");
            if (ns[0] != '/') {
                ns = nss.top() + ns;
            }
            if (ns[ns.size()-1] != '/') {
                ns = ns + '/';
            }

            nss.push(ns);
            it = create_path(ns);
        }
    }

    if (std::string(elt.Name()) == "include") {
        file_stack.push(elt.Attribute("file"));
    }
    return true;
}

NodeListVisitor::tree_t::iterator NodeListVisitor::create_path(const std::string& path) {
    tree_t::pre_order_iterator tree_it = nodes.begin();

    size_t pos = 0;
    while (pos != std::string::npos) {
        auto new_pos = path.find('/', pos+1);
        std::string ns = path.substr(pos+1, new_pos-pos);
        pos = new_pos;
        if (ns.empty())
            continue;

        unsigned num_children = tree_it.number_of_children();
        if (num_children == 0) {
            tree_it = nodes.append_child(tree_it, NodeDesc {ns, "", "", file_stack.top(),
                                                            std::vector<param_t>(), "",
                                                            std::vector<Port>()});
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
                tree_it = nodes.append_child(tree_it, NodeDesc {ns, "", "", file_stack.top(),
                                                                std::vector<param_t>(), "",
                                                                std::vector<Port>()});
            }
        }
    }

    return tree_it;
}

void NodeListVisitor::query_topics() {
    NodeAnalyzer na;
    AnalysisOptions opts {false, false};

    for (auto it = nodes.begin(); it != nodes.end(); ++it) {
        // continue, if not a node
        if (it->type.empty())
            continue;

        //ROS_INFO_STREAM("querying topics for " << it->name);

        auto ports = na.analyze_node(*it, global_params, opts);

        for (auto& p : ports)
            p.name = get_absolute_path(it.node, p.name);

        it->ports = ports;

        //ROS_INFO_STREAM("...found " << it->ports.size() << " topics/services");
        //ROS_INFO_STREAM(*it);
    }
}
