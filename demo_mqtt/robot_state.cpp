#include "robot_state.h"

bool robot_state::_selftested = false;

state_name robot_state::transition(const std::string& event)
{
    state_name res = state_name::e_none;
    if(std::find(_action_list.begin(),_action_list.end(),event)!=_action_list.end())
        res = (this->*_action_map[event])();
    
    return res;
}

state_name robot_state::get_name()
{
    return _name;
}

state_initial::state_initial()
{
    _name = state_name::e_initial;
    action_table action_config[] = 
    {
        {"enable",      static_cast<ptr_action>(&state_initial::goto_enable_state)},
        {"stop",        static_cast<ptr_action>(&state_initial::goto_error_state)},
    };
    int count = sizeof(action_config)/sizeof(*action_config);
    while (count--)
    {
        _action_map[action_config[count].action] = action_config[count].func;
        _action_list.push_back(action_config[count].action);
    }
}

state_name state_initial::run()
{
    std::cout<<"this is initial state"<<std::endl;
    return _name;
}

state_name state_initial::goto_enable_state()
{
    return state_name::e_enable;
}

state_name state_initial::goto_error_state()
{
    return state_name::e_error;
}


///////////////////////////////////////////////////////////////////////
state_enable::state_enable()
{
    _name = state_name::e_enable;
    action_table action_config[] = 
    {
        {"disable",     static_cast<ptr_action>(&state_enable::goto_initial_state)},
        {"selftest",    static_cast<ptr_action>(&state_enable::goto_selftest_state)},
        {"stop",        static_cast<ptr_action>(&state_enable::goto_error_state)},
    };
    int count = sizeof(action_config)/sizeof(*action_config);
    while (count--)
    {
        _action_map[action_config[count].action] = action_config[count].func;
        _action_list.push_back(action_config[count].action);
    }
}

state_name state_enable::run()
{
    std::cout<<"this is enable state"<<std::endl;
    if(_selftested)
        return goto_ready_state();
        
    return _name;
}

state_name state_enable::goto_initial_state()
{
    return state_name::e_initial;
}

state_name state_enable::goto_selftest_state()
{
    return state_name::e_selftest;
}

state_name state_enable::goto_ready_state()
{
    return state_name::e_ready;
}

state_name state_enable::goto_error_state()
{
    return state_name::e_error;
}

///////////////////////////////////////////////////////////////////////
state_selftest::state_selftest()
{
    _name = state_name::e_selftest;
    _count = 0;
    action_table action_config[] = 
    {
        {"stop",        static_cast<ptr_action>(&state_selftest::goto_error_state)},
    };
    int count = sizeof(action_config)/sizeof(*action_config);
    while (count--)
    {
        _action_map[action_config[count].action] = action_config[count].func;
        _action_list.push_back(action_config[count].action);
    }
}

state_name state_selftest::run()
{
    std::cout<<"this is selftest state"<<std::endl;
    _count++;
    if(_count==10)
    {
        _count = 0;
        _selftested = true;
        return goto_ready_state();
    }
    else
        return _name;
}

state_name state_selftest::goto_ready_state()
{
    return state_name::e_ready;
}

state_name state_selftest::goto_error_state()
{
    return state_name::e_error;
}


///////////////////////////////////////////////////////////////////////
state_ready::state_ready()
{
    _name = state_name::e_ready;
    action_table action_config[] = 
    {
        {"motion",      static_cast<ptr_action>(&state_ready::goto_motion_state)},
        {"stop",        static_cast<ptr_action>(&state_ready::goto_error_state)},
    };
    int count = sizeof(action_config)/sizeof(*action_config);
    while (count--)
    {
        _action_map[action_config[count].action] = action_config[count].func;
        _action_list.push_back(action_config[count].action);
    }
}

state_name state_ready::run()
{
    std::cout<<"this is ready state"<<std::endl;
    return _name;
}

state_name state_ready::goto_motion_state()
{
    return state_name::e_auto_motion;
}

state_name state_ready::goto_error_state()
{
    return state_name::e_error;
}


///////////////////////////////////////////////////////////////////////
state_motion::state_motion()
{
    _name = state_name::e_auto_motion;
    _count = 0;
    action_table action_config[] = 
    {
        {"stop",        static_cast<ptr_action>(&state_motion::goto_error_state)},
    };
    int count = sizeof(action_config)/sizeof(*action_config);
    while (count--)
    {
        _action_map[action_config[count].action] = action_config[count].func;
        _action_list.push_back(action_config[count].action);
    }
}

state_name state_motion::run()
{
    std::cout<<"this is motion state"<<std::endl;
    _count++;
    if(_count==10)
    {
        _count = 0;
        return goto_ready_state();
    }
    else
        return _name;
}

state_name state_motion::goto_ready_state()
{
    return state_name::e_ready;
}

state_name state_motion::goto_error_state()
{
    return state_name::e_error;
}

///////////////////////////////////////////////////////////////////////
state_error::state_error()
{
    _name = state_name::e_error;
    action_table action_config[] = 
    {
        {"recover",        static_cast<ptr_action>(&state_error::goto_initial_state)},
    };
    int count = sizeof(action_config)/sizeof(*action_config);
    while (count--)
    {
        _action_map[action_config[count].action] = action_config[count].func;
        _action_list.push_back(action_config[count].action);
    }
}

state_name state_error::run()
{
    std::cout<<"this is error state"<<std::endl;
    return _name;
}

state_name state_error::goto_initial_state()
{
    return state_name::e_initial;
}


///////////////////////////////////////////////////////////////////////
state_machine::state_machine()
{
    struct state_table
    {
        state_name  name;
        robot_state* state;
    }state_config[] = 
    {
        {state_name::e_initial,     new state_initial()},
        {state_name::e_enable,      new state_enable()},
        {state_name::e_selftest,    new state_selftest()},
        {state_name::e_ready,       new state_ready()},
        {state_name::e_auto_motion, new state_motion()},
        {state_name::e_error,       new state_error()},
    };
    int count = sizeof(state_config)/sizeof(*state_config);
    while (count--)
        _state_map[state_config[count].name] = state_config[count].state;
    
    _cur_state = _state_map[state_name::e_initial];
}

bool state_machine::handle_event(const std::string& event)
{
    auto next_state = _cur_state->transition(event);
    if(next_state!=state_name::e_none)
    {
        _cur_state = _state_map[next_state];
        return true;
    } 
    else
        return false;
}

void state_machine::run()
{
    auto next_state = _cur_state->run();
    _cur_state = _state_map[next_state];
}

state_name state_machine::get_cur_state_name()
{
    return _cur_state->get_name();
}