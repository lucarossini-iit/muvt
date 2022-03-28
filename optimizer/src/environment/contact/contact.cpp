#include <environment/contact/contact.h>

using namespace XBot::HyperGraph;

Contact::Contact(std::string distal_link):
_distal_link(distal_link)
{}

Contact Contact::operator=(const Contact &other)
{
    this->state.pose = other.state.pose;
    this->setDistalLink(other.getDistalLink());
}

std::string Contact::getDistalLink() const
{
    return _distal_link;
}

void Contact::setDistalLink(std::string distal_link)
{
    _distal_link = distal_link;
}
