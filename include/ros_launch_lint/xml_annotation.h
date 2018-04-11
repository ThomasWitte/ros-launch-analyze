#ifndef XML_ANNOTATION_H
#define XML_ANNOTATION_H

#include "ros_launch_lint/tree.h"
#include "ros_launch_lint/node_tree.h"

std::vector<Port> parse_annotation(const XMLComment& elt, NodeTree::tree_t::tree_node *tnode);

void xml_annotation(NodeTree& tree);

bool diff_ports(const NodeTree& a, const NodeTree& b, std::ostream& output);

#endif // XML_ANNOTATION_H
