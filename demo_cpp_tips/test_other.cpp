#include "test_other.h"
#include "float.h"

#define NUM_MOTORS 8

namespace test_other
{


void convert_string()
{
	json j = {
		{"name", "stop"},
		{"part", "master"},
		{"pos", {{-20.0,30, -0.04}}},
		{"type", "fixed-base"},
		{"mode", "relative"}
	};
	auto calc_rows = [](const std::string& type)->int
	{
		std::vector<std::string> type_name{"joint","vertex","fixed-base","fixed_orientation","fixed_rcm",
			"direction_base","direction_rcm"};
		int index = std::find(type_name.begin(),type_name.end(),type)-type_name.begin();
		return index==type_name.size()?-1:(index==0?5:(index==1?2:3));
	};
	auto is_valid = [j](int rows) -> bool
	{
		bool is_valid = true;
		std::vector<std::vector<double>> pos_vec = j.at("pos");
		for(int idx=0;idx<pos_vec.size();idx++)
			is_valid = is_valid && (pos_vec.at(idx).size()==rows);

		return is_valid;
	};
	std::cout<<is_valid(calc_rows(j.at("type")))<<std::endl;
	// std::cout<<calc_rows("vertex")<<std::endl;
	// std::cout<<calc_rows("fixed-base")<<std::endl;
	// std::cout<<calc_rows("aaaa")<<std::endl;
}

void json_conversion()
{
	json jn;
	double array[]{3,2,1};
	jn["pos"] = std::vector<double>{1,2,3};
	jn["vel"] = array;
	std::cout<<jn.at("pos")<<std::endl;
	std::cout<<jn.at("vel")<<std::endl;
}

void bind_function()
{
	json j = {
		{"control_word",{1,1,1,1,1,1,1,1}},
		{"control_mode",{8,8,8,8,8,8,8}},
		{"cmd_vel",{1000,1000,1000,1000,1000,1000,1000,1000}}
	};
	auto is_valid = [j](const std::string& cmd)->bool
	{
		std::vector<int> value = j.at(cmd);
		return value.size()==NUM_MOTORS?true:false;
	};
	std::cout<<is_valid("control_word")<<std::endl;
	std::cout<<is_valid("control_mode")<<std::endl;

	using func_ptr = void (*)(int);
	auto f = std::bind<func_ptr>(tt_argumetes,std::placeholders::_1);
	f(2);

	std::cout<<std::endl<<std::endl;
	test_class tclass;
	auto execute_cmd = [j](const std::string& cmd,std::function<void(std::vector<int>)> execute)
	{
		std::vector<int> value = j.at(cmd);
		if(value.size()==NUM_MOTORS)
			execute(value);
		else
			std::cout<<"input parameters is invalid"<<std::endl;
	};
	using mfunc_ptr = void(test_class::*)(std::vector<int>);
	auto p = std::bind<void(test_class::*)(std::vector<int>)>(&test_class::set_parameters,&tclass,std::placeholders::_1);
	execute_cmd("control_word",p);
	execute_cmd("control_mode",p);
	// is_valid("control_word");
}

void test_fill()
{
	int aa[4];
	std::fill(aa,aa+2,3);
	std::fill(aa+2,aa+4,5);
	for(int idx=0;idx<4;idx++)
		std::cout<<aa[idx]<<std::endl;

	std::vector<double> vd(7);
	for(auto it=vd.begin();it!=vd.end();it++)
		std::cout<<*it<<std::endl;
	std::fill(vd.begin(),vd.end(),5);
	for(auto it=vd.begin();it!=vd.end();it++)
		std::cout<<*it<<std::endl;
}

void test_time()
{
	time_t rawtime = time(nullptr);
	//time(&rawtime);
	struct tm timeinfo;
	localtime_s(&timeinfo, &rawtime);
	std::cout<<timeinfo.tm_year+1900<<std::endl;
	std::cout<<timeinfo.tm_mon+1<<std::endl;
	std::cout<<timeinfo.tm_mday<<std::endl;
	std::cout<<timeinfo.tm_hour<<std::endl;
	char time_format[20];
	strftime(time_format, 20, "%Y_%m%d_%H%M%S", &timeinfo);
	std::string timestr(time_format);
	std::cout<<timestr<<std::endl;
}

void test_enable_logic()
{
	uint32_t status_word = 112;
	uint32_t ctrl_word = 20;
	if ((status_word>>3) & 1)
		ctrl_word = 128;
	else if (((status_word & 0xf) ==7) && (((status_word>>5) & 3) ==1))
		ctrl_word = 15;
	else
	{
		if ((status_word & 0xf) ==0)
			ctrl_word = 6;
		else if (((status_word &0xf) ==1) && (((status_word>>5) & 3) ==1))
			ctrl_word = 7;
		else if (((status_word &0xf) ==3) && (((status_word>>5) & 3) ==1))
			ctrl_word = 15;
		else
			ctrl_word = 0;
	}
}

void test_uint_convert()
{
	uint8_t test_a = 0x85;
	uint8_t test_b = 0xff;
	uint16_t tt = ((uint16_t)test_b<<8)+test_a;
	int turns = math_tools::u_to_i<uint16_t>(tt);
	auto turns1 = math_tools::i_to_u<int8_t>(-100);
	//int tt = (int8_t)(test_a);
	//int turns = (int8_t)(test_a)<<0;
	uint32_t vel_tmp = 0xffffdbf3;
	auto vel = (int32_t)(vel_tmp);
	uint8_t v1 = 0xf3;
	uint8_t v2 = 0xdb;
	uint8_t v3 = 0xff;
	uint8_t v4 = 0xff;
	//uint32_t vtmp = (v4<<8*3)+(v3<<8*2)+(v2<<8*1)+(v1<<8*0);
	int32_t v = (v4<<8*3)+(v3<<8*2)+(v2<<8*1)+(v1<<8*0);

}

void test_directory()
{
#ifdef _WIN32
	char path[256];
	_getcwd(path, 256);
	std::cout<<"Current working directory: "<<path<<std::endl;
	std::string file_name = "../data/joint_position.csv";
#elif __linux__
	std::cout<<"Current working directory: "<<get_current_dir_name()<<std::endl;
	string file_name = "../../data/joint_position.csv";
#endif
}

void test_random_number()
{
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<double> dist(0, std::nextafter(1.0,DBL_MAX));
	for (int idx = 0; idx<100;idx++)
		std::cout<<dist(mt)<<std::endl;
}

void test_add_definitions()
{
#ifdef SIMULATION
    std::cout<<"simulation mode"<<std::endl;
#else
    std::cout<<"runtime mode"<<std::endl;
#endif
#ifndef hello
#define hello "hello cpp"
#endif
	std::cout<<hello<<std::endl;
}

int mul_sum(int n,...)
{
	va_list argptr;
	va_start(argptr,n);
	int sum = 0;
	for(int i=0;i<n;i++)
		sum += va_arg(argptr,int);
	va_end(argptr);
	return sum;
}
void myprint(const char* args,...)
{
	va_list argptr;
	va_list aq;
	va_start(argptr,args);
	va_copy(aq,argptr);
	char* arg = nullptr;
	while (nullptr!=(arg=va_arg(argptr,char*)))
	{
		std::cout<<arg<<std::endl;
	}
	va_end(argptr);
	while (nullptr!=(arg=va_arg(aq,char*)))
	{
		std::cout<<arg<<std::endl;
	}
	va_end(aq);
}
// void template_print(){std::cout<<"empty"<<std::endl;}
template<typename T>
void template_print(T arg){std::cout<<arg<<std::endl;}
template<typename T,typename... Args>
void template_print(T head,Args... rest)
{
	std::cout<<"parameter "<<head<<std::endl;
	template_print(rest...);
}
template<typename... Args>
void comma_print(Args... args)
{
	std::initializer_list<int>{(std::cout<<args<<",",0)...};
}
void variadic_arguments()
{
	system("chcp 65001");//解决cout输出中文乱码问题
	//可变参数1：可变参数宏
	// std::cout<<"可变参数宏"<<std::endl;
	// std::cout<<mul_sum(4,1,3,5,8)<<std::endl;
	// std::cout<<mul_sum(5,1,3,5,8)<<std::endl;//错误使用
	// myprint("1","hello","world","test",nullptr);

	//可变参数2：initializer_list标准库类型
	// std::cout<<"initializer_list标准库类型"<<std::endl;
	// auto error_msg = [](std::initializer_list<std::string> sl)
	// {
	// 	for(auto it=sl.begin();it!=sl.end();it++)
	// 		std::cout<<*it<<",";
	// 	std::cout<<std::endl;
	// };
	// std::string expected = "aaa", actual = "bbb";
	// if(expected!=actual)
	// 	error_msg({"functionX",expected,actual});
	// else
	// 	error_msg({"functionX","okay"});

	//可变参数3：可变参数模板
	template_print(1,"hello",3,"world");
	comma_print(1,"hello",3,"world");
}


}//namespace test_other



namespace math_tools
{


int sign(double x)
{
	return (x>0) ? 1 : -1;
}

template<typename T>
T limit_value(T value, T min_value, T max_value)
{
	if (value>max_value)
		return max_value;
	else if (value<min_value)
		return min_value;
	else
		return value;
}

template<typename T>
T limit_min_value(T value, T min_value)
{
	return (value<min_value) ? min_value : value;
}

template<typename T>
T limit_max_value(T value, T max_value)
{
	return (value>max_value) ? max_value : value;
}

template<typename UT>
int64_t u_to_i(UT value)
{
	int sz = sizeof(value);
	if (value>>8*sz-1)
		return value-pow(2, 8*sz);
	else
		return value;
}

template<typename T>
uint64_t i_to_u(T value)
{
	int sz = sizeof(value);
	if (value>>8*sz-1)
	{
		return value+pow(2, 8*sz);
	}
	else
		return value;
}

template int limit_value(int, int, int);
template double limit_value(double, double, double);
template float limit_value(float, float, float);
template int limit_min_value(int, int);
template double limit_min_value(double, double);
template float limit_min_value(float, float);
template int limit_max_value(int, int);
template double limit_max_value(double, double);
template float limit_max_value(float, float);
template int64_t u_to_i(uint8_t);
template int64_t u_to_i(uint16_t);
template int64_t u_to_i(uint32_t);
template int64_t u_to_i(uint64_t);
template uint64_t i_to_u(int8_t);
template uint64_t i_to_u(int16_t);
template uint64_t i_to_u(int32_t);
template uint64_t i_to_u(int64_t);


}//namespace math_tools