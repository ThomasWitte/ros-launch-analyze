#ifndef REPORT_H
#define REPORT_H

#include "ros_launch_lint/node_tree.h"

void print_node_tree(const NodeListVisitor::tree_t& nodes, std::ostream &output);
void print_topics(const NodeListVisitor::tree_t& nodes, std::ostream &output);

void print_dot(const NodeListVisitor::tree_t& node_tree, std::ostream &output);

#endif // REPORT_H
