#include <iostream>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <eigen3/Eigen/Cholesky>

using namespace Eigen;
using namespace std;


VectorXd mah(MatrixXd& x, MatrixXd& xs){
    int ps = xs.cols(), ns=xs.rows();
    RowVectorXd xs_mean=xs.colwise().sum()/ns;
    MatrixXd xs_cen = (xs.rowwise()-xs_mean);
    MatrixXd x_cen = (x.rowwise()-xs_mean);
    MatrixXd w = xs_cen.transpose()*xs_cen/(ns-1);  
    MatrixXd b = MatrixXd::Identity(ps,ps); 
    w.ldlt().solveInPlace(b);
    x_cen = (x_cen*b).cwiseProduct(x_cen);
    return x_cen.rowwise().sum();
}

MatrixXd sub(MatrixXd&  x, VectorXi& s){
    int p = x.cols();
    MatrixXd xs(p+1,p);
    for(int i=0;i<s.size();i++){
        xs.row(i)=x.row(s(i));
    }
    return xs;
}

int main(){
    MatrixXd x = MatrixXd::Random(30,3);
    VectorXi s = (VectorXd::Random(4)).cast<int>();
    MatrixXd xs = sub(x,s);
    // ofstream file("test1.txt");
    // if (file.is_open()){
    //         file <<  x << '\n';
    // }
    
    cout << "m" << endl <<  mah(x,xs) << endl;
    cout << "s" << endl <<  s << endl;
    return 0;
}    