#include "ros_launch_lint/load_roslaunch.h"
#include "ros_launch_lint/node_tree.h"
#include "ros_launch_lint/report.h"
#include "ros_launch_lint/sandboxed_execution.h"
#include "ros_launch_lint/xml_annotation.h"
#include <iostream>
#include <fstream>

void print_usage() {
    ROS_INFO("usage: ros-launch-lint [OPTIONS] LAUNCH-FILE");
}

struct Options {
    Options() {}
    Options(int argc, char* argv[]) {
        for (int i = 1; i < argc; ++i) {
            std::string s = argv[i];
            if (s == "--print-dot")          print_dot = true;
            if (s == "--print-node-tree")    print_node_tree = true;
            if (s == "--print-topics")       print_topics = true;
            if (s == "--no-print-dot")       print_dot = false;
            if (s == "--no-print-node-tree") print_node_tree = false;
            if (s == "--no-print-topics")    print_topics = false;

            if (s == "--analyze-sandbox")    analyze_sandbox = true;
            if (s == "--no-analyze-sandbox") analyze_sandbox = false;
            if (s == "--xml-annotation")     xml_annotation = true;
            if (s == "--no-xml-annotation")  xml_annotation = false;

            if (s == "--compare") {
                print_dot = false;
                print_node_tree = false;
                print_topics = false;

                xml_annotation = false;
                analyze_sandbox = false;

                compare = true;
            }

            if (s == "-o" && i+1 < argc) {
                of.open(argv[i+1]);
            }
        }
    }

    ~Options() {
        if (of.is_open())
            of.close();
    }

    bool print_dot = false;
    bool print_node_tree = true;
    bool print_topics = true;

    bool xml_annotation = true;
    bool analyze_sandbox = true;

    bool compare = false;

    std::ostream& output() {
        if (of.is_open())
            return of;
        return std::cout;
    }

private:
    std::ofstream of;
};

int main(int argc, char* argv[]) {

    if (argc < 2) {
        print_usage();
        return 1;
    }
    
    Options op {argc-1, argv};
    auto filename = argv[argc-1];

    // load the launch file and resolve all includes and substitutions
    XMLDocument idoc, doc;
    doc.LoadFile(filename);
    include_all(idoc, resolve_roslaunch_substitutions(doc));

    // create node tree
    NodeListVisitor nv(filename);
    idoc.Accept(&nv);
    auto node_tree = nv.node_tree();

    // decorate node tree with data from the xml annotations
    if (op.xml_annotation)
        xml_annotation(node_tree);

    // decorate node tree with data from the sandboxed execution
    if (op.analyze_sandbox)
        sandboxed_execution(node_tree);

    // create report
    if (op.print_dot)
        print_dot(node_tree, op.output());

    if (op.print_node_tree)
        print_node_tree(node_tree, op.output());

    if (op.print_topics)
        print_topics(node_tree, op.output());

    // compare feature to check differences between xml annotations
    // and sandboxed execution results
    if (op.compare) {
        auto xml_tree = nv.node_tree();
        xml_annotation(xml_tree);

        auto sandbox_tree = nv.node_tree();
        sandboxed_execution(sandbox_tree);

        if (!diff_ports(xml_tree, sandbox_tree, op.output()))
            return -1;
    }

    return 0;
}
