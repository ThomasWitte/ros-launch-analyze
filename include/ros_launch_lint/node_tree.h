#ifndef NODE_TREE_H
#define NODE_TREE_H

#include "ros_launch_lint/tree.h"
#include "ros_launch_lint/sandboxed_execution.h"
#include <procxx/include/process.h>

#include <ros/ros.h>
#include <tinyxml2.h>
#include <iostream>
#include <regex>
#include <sstream>

using namespace tinyxml2;

class NodeListVisitor : public XMLVisitor {
public:

    using tree_t = tree<NodeDesc>;

    NodeListVisitor(const std::string& root_xml);

    virtual bool Visit (const XMLComment &elt) override;

    virtual bool VisitExit (const XMLElement &elt) override;

    virtual bool VisitEnter (const XMLElement &elt, const XMLAttribute *) override;

    tree_t::iterator create_path(const std::string& path);

    void query_topics();

    template <typename T>
    std::string get_absolute_path(T* node, std::string path) {
        while (path.empty() || path[0] != '/') {
            node = node->parent;
            path = node->data.name + path;
        }
        return path;
    }

    const tree_t& node_tree() const {
        return nodes;
    }

private:
    std::stack<std::string> nss;
    std::stack<std::string> file_stack;
    tree_t nodes;
    tree_t::iterator it;

    std::vector<param_t> private_params;
    std::vector<param_t> global_params;
    std::vector<Port> node_ports;
};

#endif // NODE_TREE_H
