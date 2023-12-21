#pragma once

#ifdef _WIN32
#include <Windows.h>
#include <direct.h>
#include <io.h>
#include <conio.h>
#elif __linux__
#include <sys/stat.h>
#include <unistd.h>
#endif
#include <thread>
#include <mutex>
#include <atomic>
#include <future>
#include <iostream>
#include <random>
#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include <functional>
#include <stdarg.h>
#include <initializer_list>
#include "json.hpp"
#include <Eigen/Dense>

using nlohmann::json;


enum error_algorithm
{
	eTranscendental,
	eSingularity,
	eOutofRange,
	eErrLspb
};

namespace test_other
{


void test_arguments();
void test_thread_name();
void test_future();
void test_lock_guard();
void test_mutex();
void test_thread();
void test_time();
void test_enable_logic();
void test_uint_convert();
void test_directory();
void test_random_number();
void test_fill();
void convert_string();
void json_conversion();
void bind_function();
void test_add_definitions();
void variadic_arguments();
void motion_command();
void try_catch();
void lambda_function_recursion();

inline void tt_argumetes(int a){std::cout<<"one argument"<<std::endl;}
inline void tt_argumetes(int a,int b){std::cout<<"two arguments"<<std::endl;}

struct st_val
{
	int val1;
	bool val2;
	double val3;
};

class interface_class
{
public:
	std::string _name;
public:
	interface_class() = default;
	virtual ~interface_class() = default;
};
class classA : public interface_class
{
public:
	int& _arg1;
	st_val* _st_ptr;
	std::shared_ptr<st_val> _st_sptr;
public:
	classA(/* args */) = default;
	classA(st_val& arg,st_val* arg_ptr=nullptr,std::shared_ptr<st_val> arg_sptr=nullptr)
		:_arg1(arg.val1),_st_ptr(arg_ptr),_st_sptr(arg_sptr)
	{
		_name = "classA";
		std::cout<<_name<<": "<<_arg1<<std::endl;
		std::cout<<"address: "<<&_arg1<<std::endl;
	}
	~classA() = default;
	void set_parameters(std::vector<int> v)
	{
		std::cout<<"complete setting parameters"<<std::endl;
	}
	void set_parameters(int v)
	{
		std::cout<<"complete setting parameters"<<std::endl;
	}
	void change()
	{
		_arg1 = 1;
	}
};
class classB : public interface_class
{
public:
	int& _arg1;
	st_val* _st_ptr;
	std::shared_ptr<st_val> _st_sptr;
public:
	classB() = default;
	classB(st_val& arg,st_val* arg_ptr=nullptr,std::shared_ptr<st_val> arg_sptr=nullptr)
		:_arg1(arg.val1),_st_ptr(arg_ptr),_st_sptr(arg_sptr)
	{
		_name = "classB";
		std::cout<<_name<<": "<<_arg1<<std::endl;
		std::cout<<"address: "<<&_arg1<<std::endl;	
	}
	~classB() = default;
};
class manager
{
public:
	std::shared_ptr<classA> _ca;
	std::shared_ptr<classB> _cb;
	st_val stval;
public:
	manager()
	{
		// st_val stval{0,false,3.3};
		stval = st_val{0,false,3.3};
		// st_val stval1{100,true,2.2};
		st_val* stval1 = new st_val();
		stval1->val1 = 100;
		stval1->val2 = true;
		stval1->val3 = 2.2;
		std::shared_ptr<st_val> stval2 = std::make_shared<st_val>();
		stval2->val1 = 200;
		stval2->val2 = true;
		stval2->val3 = 4.4;
		_ca = std::make_shared<classA>(stval,stval1,stval2);
		_cb = std::make_shared<classB>(stval,stval1,stval2);
	}
	void run(st_val& stval)
	{
		// _ca = std::make_shared<classA>(stval);
		// _cb = std::make_shared<classB>(stval);
		while (true)
		{
			std::this_thread::sleep_for(std::chrono::seconds(1));
			_ca->change();
			// std::cout<<_ca->_name<<": "<<_ca->_arg1<<std::endl;
			// std::cout<<"address: "<<&(_ca->_arg1)<<std::endl;
			// std::cout<<_cb->_name<<": "<<_cb->_arg1<<std::endl;
			// std::cout<<"address: "<<&(_cb->_arg1)<<std::endl;
		}
	}
	~manager() = default;
};

	
}


namespace math_tools
{


int sign(double x);

template<typename T>
T limit_value(T value, T min_value, T max_value);

template<typename T>
T limit_min_value(T val, T min_value);

template<typename T>
T limit_max_value(T value, T max_value);

template<typename UT>
int64_t u_to_i(UT value);

template<typename T>
uint64_t i_to_u(T value);


}