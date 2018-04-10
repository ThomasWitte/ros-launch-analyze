#ifndef XML_ANNOTATION_H
#define XML_ANNOTATION_H

#include "ros_launch_lint/tree.h"
#include "ros_launch_lint/node_tree.h"

std::vector<Port> parse_annotation(const XMLComment& elt);

void xml_annotation(NodeTree& tree);

#endif // XML_ANNOTATION_H
