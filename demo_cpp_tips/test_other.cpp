#include "test_other.h"
#include "float.h"

namespace test_other
{
	void test_other()
	{
        //test_random_number();
        test_uint_convert();
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
}



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
}