#include <teb_test/environment/contact/contact.h>

using namespace XBot::HyperGraph;

Contact::Contact(std::string distal_link):
_distal_link(distal_link)
{}

Contact Contact::operator=(const Contact &other)
{
    this->state.pose = other.state.pose;
    this->state.zmp = other.state.zmp;
    this->state.cp = other.state.cp;
    this->state.time = other.state.time;
    this->setDistalLink(other.getDistalLink());
    return *this;
}

std::string Contact::getDistalLink() const
{
    return _distal_link;
}

void Contact::setDistalLink(std::string distal_link)
{
    _distal_link = distal_link;
}

void Contact::print()
{
    std::cout << "---------------------" << std::endl;
    std::cout << "- DISTAL LINK: " << _distal_link << std::endl;
    std::cout << "- POSE: \n " << state.pose.matrix() << std::endl;
    std::cout << "---------------------" << std::endl;
}
