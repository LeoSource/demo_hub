#define _USE_MATH_DEFINES
#include "test_eigen.h"
#include <random>
#include "float.h"
#include "json.hpp"
#include <unsupported/Eigen/NonLinearOptimization>

using nlohmann::json;

namespace test_eigen
{


NeuralNetwork::NeuralNetwork()
{
    _input_xoffset.resize(4);
    _input_gain.resize(4);
    _output_xoffset.resize(2);
    _output_gain.resize(2);
    _b1.resize(32);
    _b2.resize(2);
    _w1.resize(32,4);
    _w2.resize(2,32);
    _input_xoffset<<-0.52618405,-0.5318202,-0.52618405,-0.5318202;
    _input_gain<<1.88956130187524,1.88104208941716,1.88956130187524,1.88104208941716;
    _output_xoffset<<-0.51504488,-0.51970131;
    _output_gain<<1.93526975521915,1.92980377369268;
    _b1<<-2.27634743868437317,2.7728767677620167476,-3.8010848452315393331,-0.048541788895079200639,
        2.6342891248003907911,-1.9279761799903718522,1.0840322541576892235,-1.4372116590682402215,
        -1.418287793588869139,1.3311375478360936153,1.4845625374193682777,0.13803649381636040605,
        0.10955468088863608289,0.46855892918268082781,0.042187975128613004749,0.12244590122727273818,
        0.17483715322656190327,0.057617669868894445806,-0.12379443024785258265,1.133510721599795712,
        -1.8721498079844529627,1.444584340139313472,1.133203687712064367,-0.14759138333189941727,
        -1.5009842086014146822,-0.79220116997705392059,-0.44859112263473627236,1.2816166633938954611,
        0.8278812123558441094,-3.8130610316545450189,-3.6384809677038427722,-2.8930549010802546483;
    _b2<<-0.88845993501662456904,0.50616684823571556073;
    _w1<<1.5463239465936677863,2.0171237551863843152,2.5186249471680177692,-0.36298200309735306757,
        -1.8567699056868907093,0.45551957252592223391,0.61130391619940782633,0.89051830704166212715,
        1.6759006496089532412,-0.036784581893339204917,1.6072563152589085522,0.91922206982539289566,
        -2.1965321350548308033,1.3336464947976875983,2.4833228418320798525,-1.7147846695779915827,
        -0.20356020037894645425,-0.89757582062514817967,-0.36676666841120741935,-1.0261777920962404931,
        1.9017314997938388199,-2.5985476136238649936,0.7659649431482307147,-0.50819398456915787499,
        -0.15259966299254157374,-0.7566840450698590903,0.17419231848620839176,0.77551960356847560174,
        -0.11694497426787399397,-0.6224709535342514144,-0.28297270285862441419,-0.71466147603123242327,
        4.92294644354208355,-1.6021223305609424425,-4.8897931050054648239,1.0025868906142709758,
        0.2309299310651064252,2.9955715750603388869,-0.22299315851744749839,-3.1110245552683473136,
        -1.8571309796877675247,-3.1470409448893263971,1.8411278403408244397,3.3456887650568614134,
        -1.8232829178856444496,-0.60790972971625301113,2.0093633660603655855,0.37058428968261669656,
        5.9124326465908110251,0.64899373978493268833,-6.0718686786165081415,-0.74167239866756584643,
        -0.012792641454432041037,-0.057301847374916899114,0.21215018229908458314,-0.10755933008963446118,
        2.2728239516087742622,-1.3973201080530250895,-2.552391027223767761,1.7659377222133931795,
        -0.038678291231267994399,-0.18694529756085384586,-0.079664046309998348683,-0.21827137712938632363,
        0.81521147459316312123,-1.9262288406983805178,1.8982625786747919783,1.3450806119899809676,
        1.9618679474640290383,-4.670093654278027806,-1.988215270789555511,4.8626287287359541978,
        -2.1996358586330124751,0.45598114217231355827,2.2773362333276119251,-0.5138707865224435567,
        0.12065129614417610671,0.77820828921917817311,0.18586514265232148557,-2.3645718818398324679,
        -0.43457898306549935796,2.0148869849055328629,-0.88635939260307350374,1.337914620279698541,
        4.5997511038303491304,-0.98935273737290396756,-4.4461953813524486279,0.80749553094127468622,
        0.16672650466610125508,0.78012563709879290119,-0.1709632323506097562,-0.80108830567763100738,
        -0.40905808652797454039,1.3057754310030089684,3.248432291982300324,-0.040362892496793138786,
        -1.9643881032955714527,-3.229248636962832375,1.9904460972035380451,3.1927615629613295134,
        -3.6472822291698432373,-2.3633439639818267786,3.6690743686984501615,2.4985967183001687175,
        -3.226807034135848351,-2.1052253038820394337,3.2268990886007480334,2.2044570550686826138,
        -0.16001829183129748224,-3.1846057791510227197,0.17987064193799576217,3.2630544997169175225,
        0.023815833349145061615,0.12923302806990075164,-0.42535351964301504335,0.23117068840207552483,
        -0.74218584677495114121,0.79046815568789141615,-1.0350268530135831035,2.0112453152135789658,
        -1.3403122822967514782,1.779062431065176586,0.97419562457228858765,1.0209623865828594269,
        0.63071324029550746015,-1.4039058765710488164,-2.3256947976549109391,-0.60763155676562996543;
    _w2<<0.00096379048346617352157,0.005926071742787034774,0.0070918833506383978718,0.33763505866106935382,
        -0.079181447708180585643,0.0017155606706343886052,0.095067006832671427374,-0.12603427189171181877,
        -0.0065344196832138783451,-0.0025613654471442169613,0.0077824109213060045989,0.019415758055832472301,
        0.013934221356025223298,3.8530303598318988989,0.33430888288482607518,-1.6437589944893680194,
        0.00099896305533052624955,0.0046390377254984672267,-0.031304391138291108587,0.0053273641855894520239,
        -0.0023173548307398577489,-0.0093256426856202414427,-0.09998453786756517081,0.00041723551922302390925,
        0.0051327141587298634809,0.02696976433050271571,-0.042587708928507418682,-0.00084977078484172222875,
        -0.88870864837617857823,0.018087337181839080458,-0.0034643862674092773014,0.0060163008186903642327,
        -0.0010306601749229145609,0.005873917716429027415,0.010736179663504603898,-0.16729523382580113733,
        -0.08633097335994337207,0.00081621662698830223931,0.5730649083856106385,-0.13716658566940506669,
        0.0029715808521275670004,-0.031629867122476305885,0.014532878935894704558,0.0057356564088247652747,
        -0.00069090697222957476228,-1.1228518549142403327,-0.16514024528704132622,-1.9263104913730395573,
        0.002266502741701548803,-0.012713261994430019239,0.0048596055832244256997,0.0021983746914554470539,
        0.0018218714514634660843,0.0011577144050303874931,-0.5482076977777353699,-0.00064275453357852505194,
        0.01221636559466234874,0.012454886072910179712,-0.02078506477399057889,0.028382486149490859101,
        0.28520988817096776247,0.014879152276818345679,-0.0051607159219393640079,0.0023297969816105674329;
    _input_ymin = -1;
    _output_ymin = -1;
}
VectorXd NeuralNetwork::getOutput(VectorXd input)
{
    VectorXd xp = mapminmax_apply(input);
    VectorXd a1 = tansig_apply(_w1*xp+_b1);
    VectorXd a2 = _w2*a1+_b2;
    VectorXd output = mapminmax_reverse(a2);
    return output;
}
VectorXd NeuralNetwork::mapminmax_apply(const VectorXd &x)
{
    VectorXd y = x-_input_xoffset;
    y = (y.array()*_input_gain.array()).matrix();
    y = (y.array()+_input_ymin).matrix();
    return y;
}
VectorXd NeuralNetwork::tansig_apply(const VectorXd &x)
{
    return (x.array().tanh()).matrix();
}
VectorXd NeuralNetwork::mapminmax_reverse(const VectorXd &y)
{
    VectorXd x = (y.array()-_output_ymin).matrix();
    x = (x.array()/_output_gain.array()).matrix();
    x = x+_output_xoffset;
    return x;
}
void test_neural_network()
{
    NeuralNetwork net;
    VectorXd x(4);
    x<<0.379248110000000,0.040738130000000,0.001409470000000,-0.101826210000000;
    VectorXd y = net.getOutput(x);
    std::cout<<y.transpose()<<std::endl;
    x<<0.236492020000000,0.152431100000000,-0.092067490000000,0.328934770000000;
    std::cout<<net.getOutput(x).transpose()<<std::endl;
}



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

template<typename T>
T hconcat(T matA,T matB)
{
	T rlt(matA.rows(),matA.cols()+matB.cols());
	rlt<<matA,matB;
	return rlt;
}
template<typename T>
T vconcat(T matA,T matB)
{
	T rlt(matA.rows()+matB.rows(),matA.cols());
	rlt<<matA,matB;
	return rlt;
}
Eigen::MatrixXi combs(const Eigen::VectorXi& v,int m)
{
	Eigen::MatrixXi rlt;
	int n = v.size();
	if(n==m)
		rlt = v.transpose();
	else if(m==1)
		rlt = v;		
	else
	{
		std::vector<Eigen::MatrixXi> P;
		if(m<n && m>1)
		{
			// int cnm = std::tgamma(n+1)/(std::tgamma(m+1)*std::tgamma(n-m+1));
			for(int k=0;k<n-m+1;k++)
			{
				Eigen::MatrixXi Q = combs(v.segment(k+1,n-k-1),m-1);
				int rows = Q.rows();
				int cols = Q.cols()+1;
				Eigen::MatrixXi Pmat(rows,cols);
				Pmat.col(0) = Eigen::MatrixXi::Ones(rows,1)*v(k);
				Pmat.rightCols(cols-1) = Q;
				P.push_back(Pmat);
			}
		}
		rlt = P[0];
		for(int idx=1;idx<P.size();idx++)
			rlt = vconcat<Eigen::MatrixXi>(rlt,P[idx]);
	}
	return rlt;
}
void test_combination()
{
	std::cout<<std::tgamma(5+1)<<std::endl;
	Eigen::VectorXi ev(4);
	ev<<1,2,3,4;
	std::cout<<combs(ev,2)<<std::endl;
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

void array2d2eigenMat()
{
    int arr[3][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    Eigen::Map<Eigen::Matrix<int, 3, 3>> matrix(*arr);
    Eigen::Matrix3i mat1 = Eigen::Map<Eigen::Matrix3i>((int*)arr);
    std::cout << "Eigen matrix:\n" << matrix << std::endl;
    arr[0][0] = 9;
    std::cout << "Eigen matrix:\n" << matrix << std::endl;
    std::cout << "Eigen matrix1:\n" << mat1 << std::endl;
}

void eigenMat2array2d()
{
    Eigen::Matrix<int, 3, 3> matrix;
    matrix << 1, 2, 3,
              4, 5, 6,
              7, 8, 9;
    int (*arr)[3] = new int[3][3];
    Eigen::Map<Eigen::Matrix<int, 3, 3>>(*arr,3,3) = matrix;
    std::cout << "2d array:\n";
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            std::cout << arr[i][j] << " ";
        }
        std::cout << std::endl;
    }
    delete[] arr;
}

void vector2d2eigenMat()
{
    std::vector<std::vector<int>> vec2d = {{1,2,3},{4,5,6},{7,8,9}};
    Eigen::MatrixXi matrix2(vec2d.size(), vec2d[0].size());
    for (int i = 0; i < vec2d.size(); ++i) {
        matrix2.row(i) = Eigen::Map<VectorXi>(vec2d[i].data(), vec2d[i].size());
    }
    std::cout << "Eigen matrix:\n" << matrix2 << std::endl;
}

void eigenMat2vector2d()
{
    Eigen::Matrix<int, 3, 3> matrix;
    matrix << 1, 2, 3,
              4, 5, 6,
              7, 8, 9;
    std::cout << "Eigen matrix:\n" << matrix << std::endl;
    vector<vector<int>> vec2d(3);
    for (int i = 0; i < matrix.rows(); ++i) {
        VectorXi tmp = matrix.row(i);//necessary copy operation
        vec2d[i] = std::vector<int>(tmp.data(), tmp.data() + tmp.size());
    }
    std::cout << "2d vector:\n";
    for (const auto& row : vec2d) {
        for (int value : row) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
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
