#ifndef NODE_TREE_H
#define NODE_TREE_H

#include "ros_launch_lint/tree.h"
#include <procxx/include/process.h>

#include <ros/ros.h>
#include <tinyxml2.h>
#include <iostream>
#include <regex>
#include <sstream>

using namespace tinyxml2;

using param_t = std::pair<std::string, std::string>;

struct Port {
    std::string name;
    std::string data_type;
    int64_t position;
    enum {NONE=0, PUBLISHER, SUBSCRIBER, SERVICE_ADVERTISE, SERVICE_CLIENT} type;
};

struct NodeDesc {
    std::string name;
    std::string type;
    std::string package;
    std::string launch_file;

    // params
    std::vector<param_t> params = std::vector<param_t>();
    std::string args = "";

    // topics
    std::vector<Port> ports = std::vector<Port>();

    // corresponding xml
    XMLConstHandle xml = XMLConstHandle {nullptr};

    friend std::ostream& operator<< (std::ostream& out, const NodeDesc& desc);
};

std::ostream& operator<< (std::ostream& out, const NodeDesc& desc);

struct NodeTree {
    using tree_t = tree<NodeDesc>;
    tree_t nodes;
    std::vector<param_t> global_params;
};

template <typename T>
std::string get_absolute_path(T* node, std::string path) {
    while (path.empty() || path[0] != '/') {
        node = node->parent;
        path = node->data.name + path;
    }
    return path;
}

class NodeListVisitor : public XMLVisitor {
public:
    NodeListVisitor(const std::string& root_xml);

    virtual bool VisitExit (const XMLElement &elt) override;

    virtual bool VisitEnter (const XMLElement &elt, const XMLAttribute *) override;

    NodeTree::tree_t::iterator create_path(const std::string& path);

    const NodeTree& node_tree() const {
        return tree;
    }

private:
    std::stack<std::string> nss;
    std::stack<std::string> file_stack;
    NodeTree tree;
    NodeTree::tree_t::iterator it;

    std::vector<param_t> private_params;
};

#endif // NODE_TREE_H
