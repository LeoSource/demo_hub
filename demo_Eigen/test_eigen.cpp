#define _USE_MATH_DEFINES
#include "test_eigen.h"
#include <random>
#include "float.h"
#include "json.hpp"
#include <unsupported/Eigen/NonLinearOptimization>

using nlohmann::json;

namespace test_eigen
{


    struct MyFunctor
    {
    private:
        Eigen::VectorXd _paramk;
        double _l0,_px,_py;
    public:
        MyFunctor(Eigen::VectorXd paramk,double l0,double px,double py)
        {
            _paramk = paramk;
            _l0 = l0;
            _px = px;
            _py = py;
        }

        int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
        {
            double w = (_paramk(0)*_l0*_l0*_l0*_l0+_paramk(1)*_l0*_l0*_l0)*x(1);
            double phi = (_paramk(2)*_l0*_l0*_l0+_paramk(3)*_l0*_l0)*x(1);
            fvec(0) = _l0*cos(x(1)-w/_l0)+x(0)*cos(x(1)-phi)-_px;
            fvec(1) = _l0*sin(x(1)-w/_l0)+x(0)*sin(x(1)-phi)-_py;
            return 0;
        }


        int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const
        {
            double phi = (_paramk(2)*_l0*_l0*_l0+_paramk(3)*_l0*_l0)*x(1);
            fjac(0,0) = cos(x(1)-phi);
            double a = 1-(_paramk(0)*_l0*_l0*_l0+_paramk(1)*_l0*_l0);
            double b = 1-(_paramk(2)*_l0*_l0*_l0+_paramk(3)*_l0*_l0);
            fjac(0,1) = -_l0*a*sin(a*x(1))-x(0)*b*sin(b*x(1));
            fjac(1,0) = sin(x(1)-phi);
            fjac(1,1) = a*_l0*cos(a*x(1))+b*x(0)*cos(b*x(1));
            return 0;
        }

        // int inputs() const { return 3; }
        int values() const { return 2; } // number of constraints
    };
    void test_nonlinear_equation()
    {
        Eigen::VectorXd x(2);
        x(0) = 0;
        x(1) = 0;

        Eigen::VectorXd paramk(4);
        paramk<<0.021192213499105,-0.087500500919441,-0.010985344150205,0.076021897320699;
        MyFunctor functor(paramk,3,8,2);
        Eigen::LevenbergMarquardt<MyFunctor, double> lm(functor);

        lm.minimize(x);
        std::cout<<x<<std::endl;
    }

    void test_eigen()
    {
        //// test transformation between rotation matrix and RPY angle
        //Eigen::Vector3d rpy(-50*D2R, 20*D2R, 10*D2R);
        //Eigen::Matrix3d rot_mat = robot_tools::rpy2rotmatrix(rpy);
        //std::cout<<rot_mat<<std::endl;
        //std::cout<<robot_tools::rotmatrix2rpy(rot_mat)*R2D<<std::endl;

        //// test rotation matrix calculation according to unit rotation
        //Eigen::Matrix3d mat = (Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX())
        //    *AngleAxisd(0, Eigen::Vector3d::UnitY())).matrix();
        //std::cout<<mat<<std::endl;
        //std::cout<<Eigen::Vector3d::UnitX()<<std::endl;

        //// test transformation between eigen matrix and array/vector
        //array2eigenMat();
        //eigenMat2array();

        //array2eigenVec();
        //eigenVec2array();

        //vector2eigenMat();
        //eigenMat2vector();

        //vector2eigenVec();
        //eigenVec2vector();

        test_random_matrix();

        std::vector<double> vec{1,2,3,4,5,6};
        Eigen::VectorXd vp = Eigen::Map<VectorXd>(vec.data(),vec.size());
        auto func = [](Eigen::VectorXd vp)->Eigen::VectorXd
        {
            Eigen::VectorXd q(5);
            q<<5,4,3,2,1;
            return q;
        };
        vp = func(vp);
        std::cout<<vp<<std::endl;
    }

    void test_lambda()
    {
        json j = {
            {"name", "stop"},
            {"part", "master"},
            {"pos", {{-20.0,30, -0.04}}},
            {"type", "joint"},
            {"mode", "relative"}
        };
        auto parse_via_pos = [j](const Eigen::VectorXd& pos_fdb,bool joint_type) -> Eigen::MatrixXd
        {
            std::vector<std::vector<double>> pos_vec = j.at("pos");
            int rows = pos_vec[0].size();
            Eigen::VectorXd pos_delta = pos_fdb;
            Eigen::MatrixXd via_pos(rows,pos_vec.size());
            std::cout<<"rows: "<<rows<<std::endl;
            for(int idx=0;idx<pos_vec.size();idx++)
            {
                Eigen::VectorXd pos_tmp;
                Eigen::Vector2d pos_vec_tmp;
                if(rows==2)
                {
                    pos_tmp.setZero(3);
                    for(int ridx=0;ridx<rows;ridx++)
                        pos_vec_tmp(ridx) = pos_vec.at(idx).at(ridx);
                    pos_tmp << pos_vec_tmp, 0;
                }
                else
                {
                    pos_tmp.setZero(rows);
                    for(int ridx=0;ridx<rows;ridx++)
                        pos_tmp(ridx) = pos_vec.at(idx).at(ridx);
                }
                if(joint_type)
                {
                    pos_tmp(rows-3) *= D2R;
                    pos_tmp(rows-2) *= D2R;
                }
                Eigen::VectorXd pos_ref = (j.at("mode")=="absolute")?(pos_tmp):(pos_tmp+pos_delta);
                pos_delta += pos_tmp;
                via_pos.col(idx) = pos_ref;
            }
            return via_pos;
        };
        Eigen::VectorXd pos_fdb = Eigen::VectorXd::Zero(3);
        std::cout<<parse_via_pos(pos_fdb,true)<<std::endl;
    }

    void test_eigen_error()
    {
        try
        {
            Eigen::VectorXd q = Eigen::VectorXd::Ones(3);
            Eigen::Vector2d mat;
            mat<<1,2,3,4;
            // Eigen::MatrixXd res = q*mat;
            Eigen::MatrixXd res1 = mat*q;
            std::cout<<res1<<std::endl;
            q(4) = 5;
            std::cout<<q(3)<<std::endl;
        }
        catch(const std::exception& e)
        {
            std::cout << e.what() << '\n';
        }
        
    }

    void array2eigenMat()
    {
        cout << "-------------------------- array2eigenMat  --------------------------" << endl;

        int array[9];
        for (int i = 0; i < 9; ++i) array[i] = i;
        cout << "array = [ "; for (int i = 0; i < 9; ++i) cout << array[i] << " "; cout << "]" << endl;

        cout << "colMajor matrix = \n" << Map<Matrix3i>(array) << endl;                     // map a contiguous array as a column-major matrix
        cout << "rowMajor matrix = \n" << Map<Matrix<int, 3, 3, RowMajor>>(array) << endl;  // map a contiguous array as a row-major matrix


        cout << "stride matrix = \n" << Map<MatrixXi, 0, OuterStride<>>(array, 3, 3, OuterStride<>(2)) << endl;
        //mapping an array while specifying an outer stride. Here, since we're mapping as a column-major matrix, 
        // 'outer stride' means the pointer increment between two consecutive columns


        Map<MatrixXi> eigMat1(array, 3, 3);
        MatrixXi      eigMat2 = Map<MatrixXi>(array, 3, 3);
        array[0] = 9;

        cout << "eigMat1 matrix = \n"; cout << eigMat1 << endl;
        cout << "eigMat2 matrix = \n"; cout << eigMat2 << endl;
        cout << "---------------------------------------------------------------------" << endl;

    }
    void eigenMat2array()
    {
        cout << "-------------------------- eigenMat2array  --------------------------" << endl;
        Matrix3d eigMat;
        eigMat <<
            1, 2, 3,
            4, 5, 6,
            7, 8, 9;
        cout << "init eigMat = \n";    cout << eigMat << endl;

        double* eigMatptr = eigMat.data();
        cout << "array = [ "; for (int i = 0; i < 9; ++i) cout << eigMatptr[i] << " "; cout << "]" << endl;

        eigMat(0, 0) = 9;
        cout << "array = [ "; for (int i = 0; i < 9; ++i) cout << eigMatptr[i] << " "; cout << "]" << endl;


        double* eigMatptrnew = new double[eigMat.size()];
        Map<MatrixXd>(eigMatptrnew, eigMat.rows(), eigMat.cols()) = eigMat;

        eigMat(2, 2) = 0;
        cout << "init matrix = \n"; cout << eigMat << endl;
        cout << "array = [ "; for (int i = 0; i < 9; ++i) cout << eigMatptr[i] << " "; cout << "]" << endl;
        cout << "---------------------------------------------------------------------" << endl;
    }

    void array2eigenVec()
    {
        cout << "-------------------------- array2eigenVec  --------------------------" << endl;

        int array[9];
        for (int i = 0; i < 9; ++i) array[i] = i;
        cout << "data array = [ "; for (int i = 0; i < 9; ++i) cout << array[i] << " "; cout << "]" << endl;

        Map<VectorXi> eigVec(array, 5);
        cout << "eigen  vector transpose = " << eigVec.transpose() << endl;
        cout << "stride vector transpose = " << Map<VectorXi, 0, InnerStride<2> >(array, 4).transpose() << endl;
        // map an array as a vector, specifying an inner stride, that is, the pointer increment between two consecutive coefficients

        array[0] = 9;
        cout << "eigen  vector transpose = " << eigVec.transpose() << endl;
        cout << "stride vector transpose = " << Map<VectorXi, 0, InnerStride<2> >(array, 4).transpose() << endl;

        cout << "---------------------------------------------------------------------" << endl;
    }
    void eigenVec2array()
    {
        cout << "-------------------------- eigenVec2array  --------------------------" << endl;
        VectorXf eigvec(5);
        eigvec << 0, 1, 2, 3, 4;
        cout << "eigen  vector transpose = " << eigvec.transpose() << endl;

        float* array = new float;
        array = eigvec.data();
        cout << "data array = [ "; for (int i = 0; i < eigvec.size(); ++i) cout << array[i] << " "; cout << "]" << endl;

        eigvec(0) = 9;
        cout << "data array = [ "; for (int i = 0; i < eigvec.size(); ++i) cout << array[i] << " "; cout << "]" << endl;

        array[0] = 5;
        cout << "eigen  vector transpose = " << eigvec.transpose() << endl;

        cout << "---------------------------------------------------------------------" << endl;
    }

    void vector2eigenMat()
    {
        cout << "-------------------------- vector2eigenMat --------------------------" << endl;
        vector<int> stdvec{ 1, 2, 3, 4, 5, 6, 7, 8, 9 };
        Map<Matrix<int, 3, 3, RowMajor>> eigMat1(stdvec.data());
        MatrixXi                         eigMat2 = Map<Matrix<int, 3, 3, RowMajor>>(stdvec.data());

        cout << "eigMat1 matrix = \n"; cout << eigMat1 << endl;
        cout << "eigMat2 matrix = \n"; cout << eigMat2 << endl;

        stdvec[0] = 9;
        cout << "eigMat1 matrix = \n"; cout << eigMat1 << endl;
        cout << "eigMat2 matrix = \n"; cout << eigMat2 << endl;

        cout << "---------------------------------------------------------------------" << endl;
    }
    void eigenMat2vector()
    {
        cout << "-------------------------- eigenMat2vector --------------------------" << endl;
        Matrix3d eigMatCol;
        eigMatCol <<
            1, 2, 3,
            4, 5, 6,
            7, 8, 9;
        cout << "eigen matrix col = \n";    cout << eigMatCol << endl;
        vector<double> stdvec1(eigMatCol.data(), eigMatCol.data() + eigMatCol.size());
        cout << "std   vector1 = ["; for (int i = 0; i < stdvec1.size(); ++i) cout << stdvec1[i] << " "; cout << "]" << endl;

        Matrix<double, 3, 3, RowMajor> eigMatRow = eigMatCol;
        cout << "eigen matrix row = \n";    cout << eigMatCol << endl;
        vector<double> stdvec2(eigMatRow.data(), eigMatRow.data() + eigMatRow.size());
        cout << "std   vector2 = ["; for (int i = 0; i < stdvec2.size(); ++i) cout << stdvec2[i] << " "; cout << "]" << endl;

        cout << "---------------------------------------------------------------------" << endl;
    }

    void vector2eigenVec()
    {
        cout << "-------------------------- vector2eigenVec --------------------------" << endl;
        vector<int> stdvec{ 1, 2, 3, 4, 5 };
        cout << "std   vector = ["; for (int i = 0; i < stdvec.size(); ++i) cout << stdvec[i] << " "; cout << "]" << endl;

        Map<VectorXi> eigVec1(stdvec.data(), stdvec.size());
        VectorXi eigVec2 = Map<VectorXi>(stdvec.data(), stdvec.size());
        cout << "eigen  vector1 transpose = " << eigVec1.transpose() << endl;
        cout << "eigen  vector2 transpose = " << eigVec2.transpose() << endl;
        cout << "stride vector  transpose = " << Map<VectorXi, 0, InnerStride<2> >(stdvec.data(), 2).transpose() << endl;


        stdvec[0] = 9;
        cout << "eigen  vector1 transpose = " << eigVec1.transpose() << endl;
        cout << "eigen  vector2 transpose = " << eigVec2.transpose() << endl;

        cout << "stride vector  transpose = " << Map<VectorXi, 0, InnerStride<2> >(stdvec.data(), 2).transpose() << endl;

        cout << "---------------------------------------------------------------------" << endl;
    }
    void eigenVec2vector()
    {
        cout << "-------------------------- eigenVec2vector --------------------------" << endl;
        VectorXf eigvec(5);
        eigvec << 0, 1, 2, 3, 4;
        cout << "eigen  vector transpose = " << eigvec.transpose() << endl;

        vector<float> stdvec(eigvec.data(), eigvec.data() + eigvec.size());
        cout << "std   vector = ["; for (int i = 0; i < stdvec.size(); ++i) cout << stdvec[i] << " "; cout << "]" << endl;

        eigvec(0) = 5;
        cout << "std   vector = ["; for (int i = 0; i < stdvec.size(); ++i) cout << stdvec[i] << " "; cout << "]" << endl;
        cout << "---------------------------------------------------------------------" << endl;
    }

    void test_random_matrix()
    {
        Eigen::VectorXd q = Eigen::VectorXd::Random(5);
        std::cout<<q<<std::endl;

        std::random_device rd;
        std::mt19937 mt(rd());
        std::uniform_real_distribution<double> dist(0, std::nextafter(1.0, DBL_MAX));
        Eigen::VectorXd qrand = Eigen::VectorXd::Zero(5).unaryExpr([&mt,&dist](double) {return dist(mt); });
        std::cout<<qrand<<std::endl;
    }

}
