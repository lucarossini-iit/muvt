#include <teb_test/planner/dcm_planner.h>

using namespace XBot::HyperGraph::Planner;

DCMPlanner::DCMPlanner():
_nh(""),
_nhpr("~")
{}

void DCMPlanner::init_load_config()
{
    if(!_nhpr.hasParam("optimizer_config"))
    {
        throw std::runtime_error("Mandatory private parameter 'optimizer_config' missing");
    }

    // load planner config file (yaml)
    std::string optimizer_config_string;
    _nhpr.getParam("optimizer_config", optimizer_config_string);

    auto config = YAML::Load(optimizer_config_string);

    YAML_PARSE_OPTION(config["dcm_planner"], z_com, double, 0);
    _z_com = z_com;
    if (_z_com == 0)
        throw std::runtime_error("missing mandatory argument 'z_com'!");

    YAML_PARSE_OPTION(config["dcm_planner"], T_step, double, 0);
    _T_step = T_step;
    if (_T_step == 0)
        throw std::runtime_error("missing mandatory argument 'T_step'!");

    YAML_PARSE_OPTION(config["dcm_planner"], step_size, double, 0.2);
    _step_size = T_step;
    if (_T_step == 0)
        throw std::runtime_error("missing mandatory argument 'T_step'!");
}

void DCMPlanner::GenerateSteps(int n_steps)
{
    // init steps
    Contact left_foot("l_sole"), right_foot("r_sole");
    left_foot.state.pose.translation() << 0.0, 0.1, 0.0;
    right_foot.state.pose.translation() << 0.0, -0.1, 0.0;

    // at the moment, always start walking with the right foot
    for (int i = 0; i < _step_size; i++)
    {
        right_foot.state.pose.translation().x() += _step_size;
        left_foot.state.pose.translation().y() += _step_size;

        auto footstep = (i % 2 == 0) ? right_foot : left_foot;
        _footstep_sequence.push_back(footstep);
    }
}

void DCMPlanner::ComputeZMPandCP()
{
    // start from the last step and move backwards to the first one.
    // follows the work done in 'Integration of vertical COM motion and angular momentum in an extended Capture Point tracking controller for bipedal walking'
    _footstep_sequence.back().state.zmp = _footstep_sequence.back().state.pose.translation();
    _footstep_sequence.back().state.cp = _footstep_sequence.back().state.pose.translation();

    for(int i = _footstep_sequence.size()-2; i >= 0; i--)
    {
        _footstep_sequence[i].state.zmp = _footstep_sequence[i].state.pose.translation();
        _footstep_sequence[i].state.cp = _footstep_sequence[i].state.zmp + (_footstep_sequence[i+1].state.cp - _footstep_sequence[i].state.zmp) / exp(sqrt(9.81/_z_com*_T_step));
    }
}

