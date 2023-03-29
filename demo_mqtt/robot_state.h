#pragma once

#include <string>
#include <map>
#include <list>
#include <algorithm>
#include <iostream>

enum class state_name
{
    e_initial           = 0,
    e_enable            = 1,
    e_selftest          = 2,
    e_ready             = 10,
    e_auto_motion       = 11,
    e_keyboard_motion   = 12,
    e_teleoperation     = 13,
    e_error             = 99,
    e_none              = 100
};


///////////////////////////////////////////////////////////////////////
class robot_state
{
protected:
    using ptr_action = state_name (robot_state::*)(void);
    std::map<std::string,ptr_action> _action_map;
    std::list<std::string> _action_list;
    state_name _name;
    static bool _selftested;

    struct action_table
    {std::string action; ptr_action func;};
    
public:
    robot_state(/* args */) = default;
    ~robot_state() = default;

    virtual state_name transition(const std::string& event) = 0;

    virtual state_name run() = 0;
};


///////////////////////////////////////////////////////////////////////
class state_initial : public robot_state
{
public:
    state_initial();
    ~state_initial() = default;

    state_name transition(const std::string& event) override;

    state_name run() override;
private:
    state_name goto_enable_state();
    state_name goto_error_state();
};


///////////////////////////////////////////////////////////////////////
class state_enable : public robot_state
{
public:
    state_enable();
    ~state_enable() = default;

    state_name transition(const std::string& event) override;

    state_name run() override;
private:
    state_name goto_initial_state();
    state_name goto_selftest_state();
    state_name goto_ready_state();
    state_name goto_error_state();
};


///////////////////////////////////////////////////////////////////////
class state_selftest : public robot_state
{
private:
    int _count;
public:
    state_selftest();
    ~state_selftest() = default;

    state_name transition(const std::string& event) override;

    state_name run() override;
private:
    state_name goto_ready_state();
    state_name goto_error_state();
};


///////////////////////////////////////////////////////////////////////
class state_ready : public robot_state
{
public:
    state_ready();
    ~state_ready() = default;

    state_name transition(const std::string& event) override;

    state_name run() override;
private:
    state_name goto_motion_state();
    state_name goto_error_state();
};


///////////////////////////////////////////////////////////////////////
class state_motion : public robot_state
{
private:
    int _count;
public:
    state_motion();
    ~state_motion() = default;

    state_name transition(const std::string& event) override;

    state_name run() override;
private:
    state_name goto_ready_state();
    state_name goto_error_state();
};


///////////////////////////////////////////////////////////////////////
class state_error : public robot_state
{
public:
    state_error();
    ~state_error() = default;

    state_name transition(const std::string& event) override;

    state_name run() override;
private:
    state_name goto_initial_state();
};


///////////////////////////////////////////////////////////////////////
class state_machine
{
private:
    std::map<state_name,robot_state*> _state_map;
    robot_state* _cur_state;
public:
    state_machine();
    ~state_machine() = default;

    bool handle_event(const std::string& event);

    void run();

};