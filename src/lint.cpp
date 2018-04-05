#include "ros_launch_lint/load_roslaunch.h"
#include "ros_launch_lint/node_tree.h"
#include "ros_launch_lint/report.h"

void print_usage() {
    ROS_INFO("usage: ros-launch-lint LAUNCH-FILE");
}

int main(int argc, char* argv[]) {

    if (argc < 2) {
        print_usage();
        return 1;
    }
    
    auto filename = argv[argc-1];

    // load the launch file and resolve all includes and substitutions
    XMLDocument idoc, doc;
    doc.LoadFile(filename);
    include_all(idoc, resolve_roslaunch_substitutions(doc));

    // create node tree
    NodeListVisitor nv(filename);
    doc.Accept(&nv);

    // decorate node tree with data from the sandboxed execution
    nv.query_topics();

    // create report
    print_launch_file_info(nv.node_tree());

    return 0;
}
