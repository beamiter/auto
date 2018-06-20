#include "NonlinearOptimization.h"

/*
 *GaussNewton
 */

double func(const VectorXd& input, const VectorXd& output, const VectorXd& params, double objIndex)
{
    // obj = A * sin(Bx) + C * cos(D*x) - F
    double x1 = params(0);
    double x2 = params(1);
    double x3 = params(2);

    double t = input(objIndex);
    double f = output(objIndex);

    return x1 * t * t * t + x2 * t * t + x3 * t - f;
}

//return vector make up of func() element.
VectorXd objF(const VectorXd& input, const VectorXd& output, const VectorXd& params)
{
    VectorXd obj(input.rows());
    for(int i = 0; i < input.rows(); i++)
        obj(i) = func(input, output, params, i);

    return obj;
}

//F = (f ^t * f)/2
double Func(const VectorXd& obj)
{
    return obj.squaredNorm()/2;
}

double Deriv(const VectorXd& input, const VectorXd& output, int objIndex, const VectorXd& params,
                 int paraIndex)
{
    VectorXd para1 = params;
    VectorXd para2 = params;

    para1(paraIndex) -= DERIV_STEP;
    para2(paraIndex) += DERIV_STEP;

    double obj1 = func(input, output, para1, objIndex);
    double obj2 = func(input, output, para2, objIndex);

    return (obj2 - obj1) / (2 * DERIV_STEP);
}

MatrixXd Jacobin(const VectorXd& input, const VectorXd& output, const VectorXd& params)
{
    int rowNum = input.rows();
    int colNum = params.rows();

    MatrixXd Jac(rowNum, colNum);

    for (int i = 0; i < rowNum; i++)
    {
        for (int j = 0; j < colNum; j++)
        {
            Jac(i,j) = Deriv(input, output, i, params, j);
        }
    }
    return Jac;
}

void gaussNewton(const VectorXd& input, const VectorXd& output, VectorXd& params)
{
    int errNum = input.rows();      //error  num
    int paraNum = params.rows();    //parameter  num

    VectorXd obj(errNum);

    double last_sum = 0;

    int iterCnt = 0;
    while (iterCnt < MAX_ITER)
    {
        obj = objF(input, output, params);

        double sum = 0;
        sum = Func(obj);

        cout << "Iterator index: " << iterCnt << endl;
        cout << "parameter: " << endl << params << endl;
        cout << "error sum: " << endl << sum << endl << endl;

        if (fabs(sum - last_sum) <= 1e-12)
            break;
        last_sum = sum;

        MatrixXd Jac = Jacobin(input, output, params);
        VectorXd delta(paraNum);
        delta = (Jac.transpose() * Jac).inverse() * Jac.transpose() * obj;

        params -= delta;
        iterCnt++;
    }
}



/*
 *LevenbergMarquardt
 */


double maxMatrixDiagonale(const MatrixXd& Hessian)
{
    int max = 0;
    for(int i = 0; i < Hessian.rows(); i++)
    {
        if(Hessian(i,i) > max)
            max = Hessian(i,i);
    }

    return max;
}

//L(h) = F(x) + h^t*J^t*f + h^t*J^t*J*h/2
//deltaL = h^t * (u * h - g)/2
double linerDeltaL(const VectorXd& step, const VectorXd& gradient, const double u)
{
    double L = step.transpose() * (u * step - gradient);
    return L/2;
}

void levenMar(const VectorXd& input, const VectorXd& output, VectorXd& params)
{
    int errNum = input.rows();      //error num
    int paraNum = params.rows();    //parameter num

    //initial parameter 
    VectorXd obj = objF(input,output,params);
    MatrixXd Jac = Jacobin(input, output, params);  //jacobin
    MatrixXd A = Jac.transpose() * Jac;             //Hessian
    VectorXd gradient = Jac.transpose() * obj;      //gradient

    //initial parameter tao v epsilon1 epsilon2
    double tao = 1e-3;
    long long v = 2;
    double eps1 = 1e-12, eps2 = 1e-12;
    double u = tao * maxMatrixDiagonale(A);
    bool found = gradient.norm() <= eps1;
    if(found) return;

    double last_sum = 0;
    int iterCnt = 0;

    while (iterCnt < MAX_ITER)
    {
        VectorXd obj = objF(input,output,params);

        MatrixXd Jac = Jacobin(input, output, params);  //jacobin
        MatrixXd A = Jac.transpose() * Jac;             //Hessian
        VectorXd gradient = Jac.transpose() * obj;      //gradient

        if( gradient.norm() <= eps1 )
        {
            cout << "stop g(x) = 0 for a local minimizer optimizer." << endl;
            break;
        }

        cout << "A: " << endl << A << endl; 

        VectorXd step = (A + u * MatrixXd::Identity(paraNum, paraNum)).inverse() * gradient; //negtive Hlm.

        cout << "step: " << endl << step << endl;

        if( step.norm() <= eps2*(params.norm() + eps2) )
        {
            cout << "stop because change in x is small" << endl;
            break;
        } 

        VectorXd paramsNew(params.rows());
        paramsNew = params - step; //h_lm = -step;

        //compute f(x)
        obj = objF(input,output,params);

        //compute f(x_new)
        VectorXd obj_new = objF(input,output,paramsNew);

        double deltaF = Func(obj) - Func(obj_new);
        double deltaL = linerDeltaL(-1 * step, gradient, u);

        double roi = deltaF / deltaL;
        cout << "roi is : " << roi << endl;
        if(roi > 0)
        {
            params = paramsNew;
            u *= max(1.0/3.0, 1-pow(2*roi-1, 3));
            v = 2;
        }
        else
        {
            u = u * v;
            v = v * 2;
        }

        cout << "u = " << u << " v = " << v << endl;

        iterCnt++;
        cout << "Iterator " << iterCnt << " times, result is :" << endl << endl;
    }
}


/*
 *dogleg
 */


void dogLeg(const VectorXd& input, const VectorXd& output, VectorXd& params)
{
    int errNum = input.rows();      //error num
    int paraNum = params.rows();    //parameter num

    VectorXd obj = objF(input, output, params);
    MatrixXd Jac = Jacobin(input, output, params);  //jacobin
    VectorXd gradient = Jac.transpose() * obj;      //gradient

    //initial parameter tao v epsilon1 epsilon2
    double eps1 = 1e-12, eps2 = 1e-12, eps3 = 1e-12;
    double radius = 1.0;

    bool found  = obj.norm() <= eps3 || gradient.norm() <= eps1;
    if(found) return;

    double last_sum = 0;
    int iterCnt = 0;
    while(iterCnt < MAX_ITER)
    {
        VectorXd obj = objF(input, output, params);
        MatrixXd Jac = Jacobin(input, output, params);  //jacobin
        VectorXd gradient = Jac.transpose() * obj;      //gradient

        if( gradient.norm() <= eps1 )
        {
            cout << "stop F'(x) = g(x) = 0 for a global minimizer optimizer." << endl;
            break;
        }
        if(obj.norm() <= eps3)
        {
            cout << "stop f(x) = 0 for f(x) is so small";
            break;
        }

        //compute how far go along stepest descent direction.
        double alpha = gradient.squaredNorm() / (Jac * gradient).squaredNorm();
        //compute gauss newton step and stepest descent step.
        VectorXd stepest_descent = -alpha * gradient;
        VectorXd gauss_newton = (Jac.transpose() * Jac).inverse() * Jac.transpose() * obj * (-1);

        double beta = 0;

        //compute dog-leg step.
        VectorXd dog_leg(params.rows());
        if(gauss_newton.norm() <= radius)
            dog_leg = gauss_newton;
        else if(alpha * stepest_descent.norm() >= radius)
            dog_leg = (radius / stepest_descent.norm()) * stepest_descent;
        else
        {
            VectorXd a = alpha * stepest_descent;
            VectorXd b = gauss_newton;
            double c = a.transpose() * (b - a);
            beta = (sqrt(c*c + (b-a).squaredNorm()*(radius*radius-a.squaredNorm()))-c)
                    /(b-a).squaredNorm();

            dog_leg = alpha * stepest_descent + beta * (gauss_newton - alpha * stepest_descent);

        }

        cout << "dog-leg: " << endl << dog_leg << endl;

        if(dog_leg.norm() <= eps2 *(params.norm() + eps2))
        {
            cout << "stop because change in x is small" << endl;
            break;
        }

        VectorXd new_params(params.rows());
        new_params = params + dog_leg;

        cout << "new parameter is: " << endl << new_params << endl;

        //compute f(x)
        obj = objF(input,output,params);
        //compute f(x_new)
        VectorXd obj_new = objF(input,output,new_params);

        //compute delta F = F(x) - F(x_new)
        double deltaF = Func(obj) - Func(obj_new);

        //compute delat L =L(0)-L(dog_leg)
        double deltaL = 0;
        if(gauss_newton.norm() <= radius)
            deltaL = Func(obj);
        else if(alpha * stepest_descent.norm() >= radius)
            deltaL = radius*(2*alpha*gradient.norm() - radius)/(2.0*alpha);
        else
        {
            VectorXd a = alpha * stepest_descent;
            VectorXd b = gauss_newton;
            double c = a.transpose() * (b - a);
            beta = (sqrt(c*c + (b-a).squaredNorm()*(radius*radius-a.squaredNorm()))-c)
                    /(b-a).squaredNorm();

            deltaL = alpha*(1-beta)*(1-beta)*gradient.squaredNorm()/2.0 + beta*(2.0-beta)*Func(obj);

        }

        double roi = deltaF / deltaL;
        if(roi > 0)
        {
            params = new_params;
        }
        if(roi > 0.75)
        {
            radius = max(radius, 3.0 * dog_leg.norm());
        }
        else if(roi < 0.25)
        {
            radius = radius / 2.0;
            if(radius <= eps2*(params.norm()+eps2))
            {
                cout << "trust region radius is too small." << endl;
                break;
            }
        }

        cout << "roi: " << roi << " dog-leg norm: " << dog_leg.norm() << endl;
        cout << "radius: " << radius << endl;

        iterCnt++;
        cout << "Iterator " << iterCnt << " times" << endl << endl;
    }
}