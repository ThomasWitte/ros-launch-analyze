#ifndef LOAD_ROSLAUNCH_H
#define LOAD_ROSLAUNCH_H

#include <tinyxml2.h>
#include <string>
#include <map>

using namespace tinyxml2;

class SubstitutionVisitor : public XMLVisitor {
public:
    SubstitutionVisitor(const std::map<std::string, std::string>& arg_map);

    virtual bool VisitEnter (const XMLElement& elt, const XMLAttribute *att) override;

    std::map<std::string, std::string> arg_map;

private:
    std::string resolve_roslaunch_substitutions(const std::string& str);

    std::string eval_python(std::string expression);
};

class IncludeVisitor : public XMLVisitor {
public:
    XMLNode *current_node;
    std::map<std::string, std::string> include_args;

//    virtual bool Visit( const XMLDeclaration& decl ) override {
//        XMLNode* clone = decl.ShallowClone(current_node->GetDocument());
//        current_node->InsertEndChild(clone);
//        return true;
//    }

    virtual bool Visit( const XMLText& text ) override;

    virtual bool Visit( const XMLComment& comment ) override;

    virtual bool Visit( const XMLUnknown& unknown ) override;

    virtual bool VisitEnter (const XMLElement &elt, const XMLAttribute *) override;

    virtual bool VisitExit (const XMLElement &elt) override;
};

XMLDocument& resolve_roslaunch_substitutions(
        XMLDocument& doc,
        std::map<std::string, std::string> args = std::map<std::string, std::string>());

void include_all(XMLDocument& result, const XMLDocument& doc);

#endif // LOAD_ROSLAUNCH_H
