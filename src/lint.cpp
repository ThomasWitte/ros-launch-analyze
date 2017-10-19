#include <tinyxml2.h>
#include <iostream>

using namespace tinyxml2;

void print_usage() {
    std::cout << "usage: ros-launch-lint LAUNCH-FILE" << std::endl;
}

int main(int argc, char* argv[]) {

    if (argc < 2) {
        print_usage();
        return 1;
    }
    
    XMLDocument doc;
    doc.LoadFile(argv[argc-1]);

    return 0;
}
