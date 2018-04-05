#ifndef REPORT_H
#define REPORT_H

#include "ros_launch_lint/node_tree.h"

void print_node_tree(const NodeListVisitor::tree_t& nodes);
void print_topics(const NodeListVisitor::tree_t& nodes);
void print_launch_file_info(const NodeListVisitor::tree_t& node_tree);

#endif // REPORT_H
