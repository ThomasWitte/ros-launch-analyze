#ifndef REPORT_H
#define REPORT_H

#include "ros_launch_lint/node_tree.h"

void print_node_tree(const NodeTree& tree, std::ostream &output);
void print_topics(const NodeTree& tree, std::ostream &output);

void print_dot(const NodeTree& tree, std::ostream &output);

#endif // REPORT_H
