    #include <iostream>  
    #include <g2o/core/base_vertex.h>// 顶点类型  
    #include <g2o/core/base_unary_edge.h>//一元边类型  
    #include <g2o/core/block_solver.h>//求解器的实现。主要来自choldmod, csparse。在使用g2o时要先选择其中一种。  
    #include <g2o/core/optimization_algorithm_levenberg.h>//莱文贝格－马夸特方法（Levenberg–Marquardt algorithm）能提供数非线性最小化（局部最小）的数值解。  
    #include <g2o/core/optimization_algorithm_gauss_newton.h>//高斯牛顿法  
    #include <g2o/core/optimization_algorithm_dogleg.h>//Dogleg（狗腿方法）  
    #include <g2o/solvers/dense/linear_solver_dense.h>//  
    #include <Eigen/Core>//矩阵库  
    #include <opencv2/core/core.hpp>//opencv2  
    #include <cmath>//数学库  
    #include <chrono>//时间库  
    using namespace std;   
      
    // 图优化   http://www.cnblogs.com/gaoxiang12/p/5244828.html  
    // 代码  https://github.com/RainerKuemmerle/g2o  
    // http://blog.csdn.net/u012525173/article/details/70332103  
    // http://blog.csdn.net/heyijia0327/article/details/47813405  
      
    // 待优化变量——曲线模型的顶点，模板参数：优化变量维度　和　数据类型  
    class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d>//定点类型  a b c三维变量  
    {  
    public:  
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 类成员 有Eigen  变量时需要 显示 加此句话 宏定义  
        virtual void setToOriginImpl() // 虚函数 重置  
        {  
            _estimate << 0,0,0;// 初始化定点  优化变量值初始化  
        }    
        virtual void oplusImpl( const double* update ) // 更新  
        {  
            _estimate += Eigen::Vector3d(update);//迭代更新 变量  
        }  
        //虚函数  存盘和读盘：留空  
        virtual bool read( istream& in ) {}  
        virtual bool write( ostream& out ) const {}  
    };  
      
      
    // 误差模型—— 曲线模型的边, 模板参数：观测值维度(输入的参数维度)，类型，连接顶点类型(创建的顶点)  
    // 一元边 BaseUnaryEdge<1,double,CurveFittingVertex>   
    // 二元边 BaseBinaryEdge<2,double,CurveFittingVertex>  
    // 多元边 BaseMultiEdge<>  
    class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>//基础一元 边类型  
    {  
    public:  
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW// 类成员 有Eigen  变量时需要 显示 加此句话 宏定义  
        CurveFittingEdge( double x ): BaseUnaryEdge(), _x(x) {}//初始化函数   直接赋值  _x = x  
        // 计算曲线模型误差  
        void computeError()  
        {  
            const CurveFittingVertex* v = static_cast<const CurveFittingVertex*> (_vertices[0]);//顶点  
            const Eigen::Vector3d abc = v->estimate();//获取顶点的优化变量  
            _error(0,0) = _measurement - std::exp( abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0) ) ;//一个误差项 _measurement为测量值  
        }  
        // 存盘和读盘：留空  
        virtual bool read( istream& in ) {}  
        virtual bool write( ostream& out ) const {}  
    public:  
        double _x;  // x 值， y 值为 _measurement  
    };  
      
    int main( int argc, char** argv )  
    {  
        double a=1.0, b=2.0, c=1.0;         // 真实参数值  
        int N=100;                          // 数据点  
        double w_sigma=1.0;                 // 噪声Sigma值  
        cv::RNG rng;                        // OpenCV随机数产生器  
        double abc[3] = {0,0,0};            // abc参数的估计值  
      
        vector<double> x_data, y_data;      // 数据  
          
        cout<<"generating data: "<<endl;  
        for ( int i=0; i<N; i++ )  
        {  
            double x = i/100.0;  
            x_data.push_back ( x );  
            y_data.push_back (exp ( a*x*x + b*x + c ) + rng.gaussian ( w_sigma ));//加上高斯噪声  
            cout<<x_data[i]<<"\t"<<y_data[i]<<endl;  
        }  
          
        // 构建图优化解决方案，先设定g2o  
        typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > Block;  // 每个误差项优化变量维度为3，误差值维度为1  
        // 线性方程求解器   H * Δx = −b  
        Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); // 线性方程求解器  
        // 稀疏矩阵块求解器 用于求解 雅克比J ( 得到右边 b = e转置 *  Ω * J ) 和  海塞矩阵 H  左边 H = J转置* Ω * J     
        Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器  
        // 迭代算法    梯度下降方法，从高斯牛顿GN,  莱文贝格－马夸特方法LM, 狗腿法DogLeg 中选  
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );  
        // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );  
        // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );  
        g2o::SparseOptimizer optimizer;     //稀疏 优化模型  
        optimizer.setAlgorithm( solver );   // 设置求解器  
        optimizer.setVerbose( true );       // 打开调试输出  
          
        // 往图中增加顶点  
        CurveFittingVertex* v = new CurveFittingVertex();//曲线拟合 新建 顶点类型  
        v->setEstimate( Eigen::Vector3d(0,0,0) );  
        v->setId(0);//id  
        optimizer.addVertex( v );  
          
        // 往图中增加边  
        for ( int i=0; i<N; i++ )  
        {  
            CurveFittingEdge* edge = new CurveFittingEdge( x_data[i] );//新建 边 带入 观测数据  
            edge->setId(i);//id  
            edge->setVertex( 0, v );           // 设置连接的顶点  
            edge->setMeasurement( y_data[i] ); // 观测数值  
            edge->setInformation( Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma) ); // 信息矩阵：单位阵协方差矩阵之逆 (个误差项权重)  就一个误差项_error(0,0)   
            optimizer.addEdge( edge );//添加边  
        }  
          
        // 执行优化  
        cout<<"start optimization"<<endl;  
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();//计时  
        optimizer.initializeOptimization();//初始化优化器  
        optimizer.optimize(100);//优化次数  
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();//结束计时  
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );  
        cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;  
          
        // 输出优化值  
        Eigen::Vector3d abc_estimate = v->estimate();  
        cout<<"estimated model: "<<abc_estimate.transpose()<<endl;  
          
        return 0;  
    }  