#include "ros_launch_lint/xml_annotation.h"

std::vector<Port> parse_annotation(const XMLComment& elt) {
    std::vector<Port> node_ports;

    // try to parse the comment as xml
    std::string comment = elt.Value();
    XMLDocument comment_doc;
    if (comment_doc.Parse(comment.c_str(), comment.size()) == XML_NO_ERROR) {
        const XMLNode* topic_node = comment_doc.FirstChild();
        if (!topic_node || !topic_node->ToElement() || std::string(topic_node->ToElement()->Name()) != "topics" || std::string(topic_node->ToElement()->Name()) != "services") {
            return std::vector<Port>();
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

    return node_ports;
}

void xml_annotation(NodeTree& tree) {
    for (auto it = tree.nodes.begin(); it != tree.nodes.end(); ++it) {
        XMLConstHandle xml = it->xml;
        for (auto elt = xml.FirstChildElement(); elt.ToElement() != nullptr; elt = elt.NextSiblingElement()) {
            if (elt.ToElement() && elt.ToElement()->ToComment())
                for (auto port : parse_annotation(*elt.ToElement()->ToComment()))
                    it->ports.push_back(port);
        }
    }
}
