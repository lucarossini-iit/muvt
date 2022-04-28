#include <muvt_core/environment/contact/contact.h>

using namespace Muvt::HyperGraph;

Contact::Contact(std::string distal_link):
_distal_link(distal_link)
{}

Contact Contact::operator=(const Contact &other)
{
    state.pose = other.state.pose;
    state.zmp = other.state.zmp;
    state.cp = other.state.cp;
    state.time = other.state.time;
    setDistalLink(other.getDistalLink());
    setContactSequence(other.getContactSequence());
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

void Contact::setContactSequence(const int sequence)
{
    _sequence = sequence;
}

int Contact::getContactSequence() const
{
    return _sequence;
}

void Contact::print()
{
    std::cout << "---------------------" << std::endl;
    std::cout << "- DISTAL LINK: " << _distal_link << std::endl;
    std::cout << "- POSE: \n " << state.pose.matrix() << std::endl;
    std::cout << "- SEQUENCE: " << _sequence << std::endl;
    std::cout << "zmp: " << state.zmp.transpose() << std::endl;
    std::cout << "---------------------" << std::endl;
}
