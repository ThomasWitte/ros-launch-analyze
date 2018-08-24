#include "ros_launch_lint/load_roslaunch.h"
#include <regex>
#include <ros/package.h>
#include <ros/ros.h>
#include <procxx/include/process.h>

SubstitutionVisitor::SubstitutionVisitor(const std::map<std::string, std::string>& arg_map)
        : arg_map {arg_map}
{
}

bool SubstitutionVisitor::VisitEnter (const XMLElement& elt, const XMLAttribute *att) {
    for (; att; att = att->Next()) {
        const_cast<XMLAttribute*>(att)->SetAttribute(resolve_roslaunch_substitutions(att->Value()).c_str());
    }

    // I hope this does not crash…
    if (std::string(elt.Name()) == "rosparam" && (elt.Attribute("subst_value", "true") || elt.Attribute("subst_value", "True"))) {
        std::string old_text = elt.GetText() ? elt.GetText() : "";
        ROS_DEBUG_STREAM("Found rosparam tag to substitute: " << old_text);
        const_cast<XMLElement*>(&elt)->SetText(resolve_roslaunch_substitutions(old_text).c_str());
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

std::string SubstitutionVisitor::resolve_roslaunch_substitutions(const std::string& str) {
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

std::string SubstitutionVisitor::eval_python(std::string expression) {
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

XMLDocument& resolve_roslaunch_substitutions(XMLDocument& doc, std::map<std::string, std::string> args) {
    SubstitutionVisitor sv {args};
    doc.Accept(&sv);
    return doc;
}

bool IncludeVisitor::Visit( const XMLText& text ) {
    XMLNode* clone = text.ShallowClone(current_node->GetDocument());
    current_node->InsertEndChild(clone);
    return true;
}

bool IncludeVisitor::Visit( const XMLComment& comment ) {
    XMLNode* clone = comment.ShallowClone(current_node->GetDocument());
    current_node->InsertEndChild(clone);
    return true;
}

bool IncludeVisitor::Visit( const XMLUnknown& unknown ) {
    XMLNode* clone = unknown.ShallowClone(current_node->GetDocument());
    current_node->InsertEndChild(clone);
    return true;
}

bool IncludeVisitor::VisitEnter (const XMLElement &elt, const XMLAttribute *) {
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

bool IncludeVisitor::VisitExit (const XMLElement &elt) {
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

void include_all(XMLDocument& result, const XMLDocument& doc) {
    IncludeVisitor iv;
    iv.current_node = &result;
    doc.Accept(&iv);
}
