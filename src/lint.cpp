#include "ros_launch_lint/tree.h"
#include "procxx/include/process.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <tinyxml2.h>
#include <iostream>
#include <regex>
#include <sstream>

using namespace tinyxml2;

void print_usage() {
    ROS_INFO("usage: ros-launch-lint LAUNCH-FILE");
}

class SubstitutionVisitor : public XMLVisitor {
public:
    SubstitutionVisitor(const std::map<std::string, std::string>& arg_map)
        : arg_map {arg_map}
    {
    }

    virtual bool VisitEnter (const XMLElement& elt, const XMLAttribute *att) override {
        for (; att; att = att->Next()) {
            const_cast<XMLAttribute*>(att)->SetAttribute(resolve_roslaunch_substitutions(att->Value()).c_str());
        }

        if (std::string(elt.Name()) == "arg") {
            // check if this arg tag is inside an include tag
            if (std::string(elt.Parent()->ToElement()->Name()) != "include") {
                std::string name = elt.Attribute("name");

                if (elt.Attribute("value")) {
                    arg_map[name] = elt.Attribute("value");
                }

                if (elt.Attribute("default")) {
                    if (arg_map.find(name) == arg_map.end()) {
                        arg_map[name] = elt.Attribute("default");
                    }
                }
            }
        }

        return true;
    }

    std::map<std::string, std::string> arg_map;

private:
    std::string resolve_roslaunch_substitutions(const std::string& str) {
        std::string result = str;

        std::smatch sm;
        while (std::regex_search(result, sm, std::regex ("\\$\\((\\w+)\\s+([^\\s]+)\\)"))) {
            if (sm[1] == "find") {
                std::string path = ros::package::getPath(sm[2]);
                result = sm.prefix().str() + path + sm.suffix().str();
            } else if (sm[1] == "arg") {
                if (arg_map.find(sm[2]) != arg_map.end()) {
                    result = sm.prefix().str() + arg_map[sm[2]] + sm.suffix().str();
                } else {
                    ROS_ERROR_STREAM("could not replace " << sm[0]);
                    break;
                }
            } else if (sm[1] == "eval") {
                result = sm.prefix().str() + eval_python(sm[2]) + sm.suffix().str();
            } else {
                ROS_ERROR_STREAM("unknown roslaunch substitution " << sm[1]);
                break;
            }
        }

        return result;
    }

    std::string eval_python(std::string expression) {
        std::string str = "arg=eval;print(" + expression + ")";
        for (const auto& arg : arg_map) {
            str = arg.first + "='" + arg.second + "';" + str;
        }

        procxx::process eval {"python", "-c", str};

        eval.exec();
        eval.wait();

        if (eval.code() != 0) {
            ROS_WARN_STREAM("Error evaluating " << str);
            std::string err;
            while (std::getline(eval.error(), err)) {
                ROS_WARN_STREAM(err);
            }
            return "unresolved_eval";
        }

        std::string result;
        eval.output() >> result;
        return result;
    }
};

XMLDocument& resolve_roslaunch_substitutions(XMLDocument& doc, std::map<std::string, std::string> args = std::map<std::string, std::string>()) {
    SubstitutionVisitor sv {args};
    doc.Accept(&sv);
    return doc;
}

class IncludeVisitor : public XMLVisitor {
public:
    XMLNode *current_node;
    std::map<std::string, std::string> include_args;

//    virtual bool Visit( const XMLDeclaration& decl ) override {
//        XMLNode* clone = decl.ShallowClone(current_node->GetDocument());
//        current_node->InsertEndChild(clone);
//        return true;
//    }

    virtual bool Visit( const XMLText& text ) override {
        XMLNode* clone = text.ShallowClone(current_node->GetDocument());
        current_node->InsertEndChild(clone);
        return true;
    }

    virtual bool Visit( const XMLComment& comment ) override {
        XMLNode* clone = comment.ShallowClone(current_node->GetDocument());
        current_node->InsertEndChild(clone);
        return true;
    }

    virtual bool Visit( const XMLUnknown& unknown ) override {
        XMLNode* clone = unknown.ShallowClone(current_node->GetDocument());
        current_node->InsertEndChild(clone);
        return true;
    }

    virtual bool VisitEnter (const XMLElement &elt, const XMLAttribute *) override {
        XMLNode* clone = elt.ShallowClone(current_node->GetDocument());
        current_node = current_node->InsertEndChild(clone);

        if (std::string(elt.Name()) == "include") {
            include_args.clear();
        }

        if (std::string(elt.Name()) == "arg") {
            // check if this arg tag is inside an include tag
            std::string name = elt.Attribute("name");
            if (elt.Attribute("value")) {
                include_args[name] = elt.Attribute("value");
            }
        }

        return true;
    }

    virtual bool VisitExit (const XMLElement &elt) override {
        if(std::string(elt.Name()) == "include") {
            std::string file = elt.Attribute("file");

            XMLDocument doc;
            XMLError err = doc.LoadFile(file.c_str());

            if (err == XML_SUCCESS) {
                ROS_INFO_STREAM("included file: " << file);
                // Resolve includes in included document
                resolve_roslaunch_substitutions(doc, include_args).Accept(this);
            } else {
                ROS_ERROR_STREAM("error " << err << " parsing included file: " << file);
            }
        }

        current_node = current_node->Parent();
        return true;
    }
};

void include_all(XMLDocument& result, const XMLDocument& doc) {
    IncludeVisitor iv;
    iv.current_node = &result;
    doc.Accept(&iv);
}

class NodeListVisitor : public XMLVisitor {
public:

    using param_t = std::pair<std::string, std::string>;

    struct Port {
        std::string topic;
        std::string data_type;
        int64_t position;
        enum {NONE=0, PUBLISHER, SUBSCRIBER} type;
    };

    struct NodeDesc {
        std::string name;
        std::string type;
        std::string package;
        std::string launch_file;

        // params
        std::vector<param_t> params;
        std::string args;

        // topics
        std::vector<Port> ports;

        friend std::ostream& operator<< (std::ostream& out, const NodeDesc& desc);
    };

    using tree_t = tree<NodeDesc>;

    NodeListVisitor(const std::string& root_xml) {
        nss.push("/");
        file_stack.push(root_xml);
        nodes.insert(nodes.begin(), NodeDesc {"/", "", "", root_xml, std::vector<param_t>(), "", std::vector<Port>()});
        it = create_path(nss.top());
    }

    virtual bool Visit (const XMLComment &elt) override {
        // assert that parent is a node tag
        if (elt.Parent() && elt.Parent()->ToElement() &&
            std::string(elt.Parent()->ToElement()->Name()) == "node") {

            // try to parse the comment as xml
            std::string comment = elt.Value();
            XMLDocument comment_doc;
            if (comment_doc.Parse(comment.c_str(), comment.size()) == XML_NO_ERROR) {
                const XMLNode* topic_node = comment_doc.FirstChild();
                if (!topic_node || !topic_node->ToElement() || std::string(topic_node->ToElement()->Name()) != "topics") {
                    return true;
                }

                for (const XMLNode* node = topic_node->FirstChild(); node != nullptr; node = node->NextSibling()) {
                    const XMLElement* elt = node->ToElement();

                    if (elt) {
                        std::string cls = elt->Attribute("class");
                        Port p {elt->Attribute("name"),
                                elt->Attribute("type"),
                                -1,
                                (cls == "pub" ? Port::PUBLISHER
                                              : (cls == "sub" ? Port::SUBSCRIBER
                                                              : Port::NONE))};
                        node_ports.push_back(p);
                    }
                }
            }
        }

        return true;
    }

    virtual bool VisitExit (const XMLElement &elt) override {
        if (std::string(elt.Name()) == "group" ||
            std::string(elt.Name()) == "node" ||
            std::string(elt.Name()) == "test" ||
            std::string(elt.Name()) == "include") {
            if (elt.Attribute("ns")) {
                nss.pop();
                it = create_path(nss.top());
            }
        }

        if (std::string(elt.Name()) == "include") {
            file_stack.pop();
        }

        if (std::string(elt.Name()) == "node") {
            nodes.append_child(it, NodeDesc {elt.Attribute("name"),
                                             elt.Attribute("type"),
                                             elt.Attribute("pkg"),
                                             file_stack.top(),
                                             private_params,
                                             elt.Attribute("args") ? elt.Attribute("args") : "",
                                             node_ports});
            private_params.clear();
            node_ports.clear();
        }

        return true;
    }

    virtual bool VisitEnter (const XMLElement &elt, const XMLAttribute *) override {
        if (std::string(elt.Name()) == "param") {
            if (elt.Attribute("name") && elt.Attribute("value")) {
                private_params.push_back(std::make_pair(elt.Attribute("name"),
                                                        elt.Attribute("value")));
            }
        }

        if (std::string(elt.Name()) == "group" ||
            std::string(elt.Name()) == "node" ||
            std::string(elt.Name()) == "test" ||
            std::string(elt.Name()) == "include") {

            if (elt.Attribute("ns")) {
                std::string ns = elt.Attribute("ns");
                if (ns[0] != '/') {
                    ns = nss.top() + ns;
                }
                if (ns[ns.size()-1] != '/') {
                    ns = ns + '/';
                }

                nss.push(ns);
                it = create_path(ns);
            }
        }

        if (std::string(elt.Name()) == "include") {
            file_stack.push(elt.Attribute("file"));
        }
        return true;
    }

    tree_t::iterator create_path(const std::string& path) {
        tree_t::pre_order_iterator tree_it = nodes.begin();

        size_t pos = 0;
        while (pos != std::string::npos) {
            auto new_pos = path.find('/', pos+1);
            std::string ns = path.substr(pos+1, new_pos-pos);
            pos = new_pos;
            if (ns.empty())
                continue;

            unsigned num_children = tree_it.number_of_children();
            if (num_children == 0) {
                tree_it = nodes.append_child(tree_it, NodeDesc {ns, "", "", file_stack.top(),
                                                                std::vector<param_t>(), "",
                                                                std::vector<Port>()});
            } else {
                bool found = false;
                for (auto child_it = tree_it.begin(); child_it != tree_it.end(); ++child_it) {
                    if (child_it->name == ns) {
                        tree_it = child_it;
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    tree_it = nodes.append_child(tree_it, NodeDesc {ns, "", "", file_stack.top(),
                                                                    std::vector<param_t>(), "",
                                                                    std::vector<Port>()});
                }
            }
        }

        return tree_it;
    }

    void query_topics() {
        for (auto it = nodes.begin(); it != nodes.end(); ++it) {
            // continue, if not a node
            if (it->type.empty())
                continue;

            ROS_INFO_STREAM("querying topics for " << it->name);

            procxx::process get_topics {"firejail",
                                        "--env=LD_LIBRARY_PATH=/home/thomas/catkin_ws/devel/lib:/home/thomas/ros_ws/devel/lib:/opt/ros/kinetic/lib:/home/thomas/local/lib",
                                        "--overlay",
                                        "--quiet",
                                        ros::package::getPath("ros_launch_lint") + "/get_topics.sh",
                                        it->package,
                                        it->type};

            std::cout << "cmd: firejail --env=LD_LIBRARY_PATH=$LD_LIBRARY_PATH --overlay --quiet " << ros::package::getPath("ros_launch_lint") << "/get_topics.sh"
                      << " " << it->package << " " << it->type;

            // split arguments (TODO: don't split quoted strings)
            if (!it->args.empty()) {
                std::stringstream ss {it->args};
                std::string s;
                while (ss >> s) {
                    get_topics.add_argument(s);
                    std::cout << " " << s;
                }
            }

            for (const auto param : it->params) {
                get_topics.add_argument(std::string("_") + param.first + ":=" + param.second);
                std::cout << " _" << param.first << ":=" << param.second;
            }
            std::cout << std::endl;

            get_topics.exec();

            std::string dir, name, type;
            while (get_topics.output() >> dir >> name >> type) {
                Port p;
                p.topic = get_absolute_path(it.node, name);
                p.data_type = type;
                p.type = (dir == "<<advertise>>" ? Port::PUBLISHER
                                                 : (dir == "<<subscribe>>" ? Port::SUBSCRIBER
                                                                           : Port::NONE));
                it->ports.push_back(p);
            }

            std::string line;
            while (std::getline(get_topics.error(), line))
                std::cout << line << std::endl;

            ROS_INFO_STREAM("...found " << it->ports.size() << " topics");
        }
    }

    template <typename T>
    std::string get_absolute_path(T* node, std::string path) {
        while (path[0] != '/') {
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
    std::vector<Port> node_ports;
};

std::ostream& operator<< (std::ostream& out, const NodeListVisitor::NodeDesc& desc) {
    out << desc.name << std::endl;

    for(const auto& p : desc.params)
        out << "  " << p.first << "=" << p.second << std::endl;

    out << std::endl;

    for(const auto& p : desc.ports)
        out << "  " << p.topic << " " << p.data_type << " " << p.type << std::endl;

    return out;
}

void print_node_tree(const NodeListVisitor::tree_t& nodes) {
    std::map<std::string, int> colors;
    int next_color = 31;
    for (auto it = nodes.begin(); it != nodes.end(); ++it) {
        for (int i = nodes.depth(it); i > 0; --i) {
            std::cout << "  ";
        }
        if (colors.find(it->launch_file) == colors.end()) {
            colors[it->launch_file] = next_color++;
        }

        std::string str = it->name;
        if (!it->type.empty()) {
            int padding_w = 32 - it->name.size() - 2*nodes.depth(it);
            if (padding_w < 0) {
                str.resize(std::max(str.size() + padding_w - 3, 0ul));
                str += "...";
                padding_w = 0;
            }

            std::string padding(padding_w, ' ');
            str += padding + "[" + it->package + "/" + it->type + "]";
        }

        std::cout << "\x1b[" << colors[it->launch_file] << "m"
                  << str
                  << "\x1b[0m" << std::endl;
    }

    std::cout << std::endl;

    for (const auto& elt : colors) {
        std::cout << "\x1b[" << elt.second << "m" << elt.first << "\x1b[0m" << std::endl;
    }
}

void print_topics(const NodeListVisitor::tree_t& nodes) {
    using Port = NodeListVisitor::Port;
    //       topic                   publisher                 subscriber                type
    std::map<std::string, std::tuple<std::vector<std::string>, std::vector<std::string>, std::string>> topics;

    for (auto it = nodes.begin(); it != nodes.end(); ++it) {
        for (const auto& p : it->ports) {
            if (std::get<2>(topics[p.topic]) != p.data_type && p.data_type != "*") {
                if (std::get<2>(topics[p.topic]) != "" &&
                    std::get<2>(topics[p.topic]) != "*") {

                    std::cout << "Incompatible types at topic " << p.topic << ": "
                              << std::get<2>(topics[p.topic]) << "!=" << p.data_type << std::endl;
                } else {
                    std::get<2>(topics[p.topic]) = p.data_type;
                }
            }

            if (p.type == Port::PUBLISHER)
                std::get<0>(topics[p.topic]).push_back(it->name);
            if (p.type == Port::SUBSCRIBER)
                std::get<1>(topics[p.topic]).push_back(it->name);
        }
    }

    for (const auto& elt : topics) {
        std::cout << elt.first << " (" << std::get<2>(elt.second) << "):" << std::endl;
        std::cout << "  ";
        for (const auto& s : std::get<0>(elt.second))
            std::cout << s << " ";
        std::cout << "-> ";
        for (const auto& s : std::get<1>(elt.second))
            std::cout << s << " ";
        std::cout << std::endl;
    }
}

void print_launch_file_info(const XMLDocument& doc, const std::string& filename) {
    NodeListVisitor nv(filename);
    doc.Accept(&nv);
    nv.query_topics();
    print_node_tree(nv.node_tree());
    print_topics(nv.node_tree());
}



int main(int argc, char* argv[]) {

    if (argc < 2) {
        print_usage();
        return 1;
    }
    
    XMLDocument idoc, doc;
    doc.LoadFile(argv[argc-1]);

    include_all(idoc, resolve_roslaunch_substitutions(doc));

    //idoc.Print();

    print_launch_file_info(idoc, argv[argc-1]);

    return 0;
}
