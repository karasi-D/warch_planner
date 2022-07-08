#include <iostream>
#include <traj_utils/polynomial_traj.h>

Eigen::MatrixXd PolynomialTraj::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::Vector3d &start_vel,
    const Eigen::Vector3d &end_vel, 
    const Eigen::Vector3d &start_acc,
    const Eigen::Vector3d &end_acc,
    const Eigen::VectorXd &Time) // time allocation in each segment
{
  // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
  int p_order   = 2 * d_order - 1;              // the order of polynomial
  int p_num1d   = p_order + 1;                  // the number of variables in each segment

  int m = Time.size();                          // the number of segments
  
  MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);   // 多项式系数，position(x,y,z), so we need (3 * p_num1d) coefficients
  VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);
  //int p_memory = m * p_num1d; // 总系数

  std::cout << "  [PolyQPGeneration] d_order = " << d_order << "; Time.size = " << m <<"; cal begine ..."<< endl;
  // dxq'code is following:
  for(int idx = 0; idx < Path.cols(); ++idx){ // 维度数量
      VectorXd startpose = VectorXd::Zero(d_order), endpose = VectorXd::Zero(d_order);
      startpose(0) = Path(0, idx);
      startpose(1) = start_vel(idx);
      startpose(2) = start_acc(idx);
      endpose(0) = Path(m, idx);
      endpose(1) = end_vel(idx);
      endpose(2) = end_acc(idx);
      /*   Produce Mapping Matrix A to the entire trajectory, A is a mapping matrix that maps polynomial coefficients to derivatives.   */
          // 即 [d] = Ct[df, dp]
          // df-fixed derivatives: fixed start, intermediate connections, goal state
          // dp-free derivatives: all derivatives at intermediate connections，所有中间点只保留结束状态！
      //std::cout << "    creat Ct begine ..."<< endl;
      MatrixXd Ct;
      Ct.resize((2*d_order)*m, d_order*(m+1));
      Ct.setZero((2*d_order)*m, d_order*(m+1));
      ////std::cout << "    cal Ct begine ..."<< endl;
      for(int k = 0; k < d_order; ++k){
          Ct(k,k) = 1; // d-df: fixed start
          Ct(2*d_order*(m-1) + d_order + k, d_order+m-1+k) = 1; // d-df: goal state
      }
      for(int j = 0; j < m-1; ++j){
          Ct(j*p_num1d + d_order, d_order+j) = 1; // d中的中间结束状态 = df中间结束状态，(0)次幂
          // d-dp项的转换矩阵
          Ct(p_num1d*(j+1), d_order+j) = 1; // d中间开始状态 = df中间结束状态,(0)幂
          for(int dk = 0; dk < d_order-1; ++ dk){
              Ct(j*p_num1d + d_order + dk+1, p_num1d+m-1 + j*(d_order-1)+dk) = 1; // d中间结束状态 = dp中间结束状态,(123)幂
              Ct(p_num1d*(j+1) + dk+1, p_num1d+m-1 + j*(d_order-1)+dk) = 1; // d中间开始状态 = dp中间结束状态,(123)幂
          }
      }
      //std::cout << "    cal Ct done, size : "<<  Ct.rows() << '*' << Ct.cols() << endl;
      
      /*   Produce the dereivatives in X, Y and Z axis directly.  M，p 阶数 升幂排序*/
      MatrixXd M;
      M.resize((2* d_order) * m, p_num1d * m);
      M.setZero((2* d_order) * m, p_num1d * m);
      // MatrixXd M = MatrixXd::Zero((2* d_order) * m, p_num1d * m);
      for(int seg = 0; seg < m; ++seg){
          for(int k = 0; k < d_order; ++k){
              M(seg * p_num1d + k, seg * p_num1d + k) = myFactorial(k);
              for(int i = k; i < p_num1d; ++i){
                  M(seg * p_num1d + d_order + k, seg * p_num1d + i) = myFactorial(i) / myFactorial(i - k) * pow(Time(seg), i - k);
              }
          }
      }
      //std::cout << "    cal M done, size : "<<  M.rows() << '*' << M.cols() << endl;
      
      //std::cout << "    creat Q begine ..."<< endl;
      /*   Produce the Minimum Snap cost function, the Hessian Matrix   */
      MatrixXd Q;
      Q.resize(m*p_num1d, m*p_num1d);
      Q.setZero(m*p_num1d, m*p_num1d);
      // MatrixXd Q = MatrixXd::Zero(m*p_num1d, m*p_num1d);
      for(int seg = 0; seg < m; ++seg){
          for(int i = 4; i <= p_order; ++i){
              for(int l = 4; l <= p_order; ++l){
                  Q(seg*p_num1d + i, seg*p_num1d + l) = i*(i-1)*(i-2)*(i-3)*l*(l-1)*(l-2)*(l-3) / (i+l-p_order) * pow(Time(seg), i+l-p_order);
              }
          }
      }
      //std::cout << "    cal Q done, size : "<<  Q.rows() << '*' << Q.cols() << endl;

      // define R, df, dp
      MatrixXd R = Ct.transpose() * (M.inverse()).transpose() * Q * M.inverse() * Ct;
      VectorXd df = VectorXd::Zero(2*d_order + m-1);
      df.segment(0, d_order) = startpose;
      df.segment(d_order+m-1, d_order) = endpose;
      for(int seg = 0; seg < m-1; ++seg){
          df(d_order+seg) = Path(seg+1, idx);
      }
      //std::cout << "    cal R done, size : "<<  Q.rows() << '*' << Q.cols() << endl;
      VectorXd dp = VectorXd::Zero((d_order-1)*(m-1));
      
      //std::cout << "    cal best_dfp begine ... "<< endl;
      MatrixXd Rpp = R.block(2*d_order+m-1, 2*d_order+m-1, (d_order-1)*(m-1), (d_order-1)*(m-1));
      //MatrixXd Rff = R.block(0,0, 2*d_order+m-1, 2*d_order+m-1);
      MatrixXd Rfp = R.block(0, 2*d_order+m-1, 2*d_order+m-1, (d_order-1)*(m-1));
      VectorXd best_dp = -Rpp.inverse() * Rfp.transpose() * df;
      VectorXd dfp = VectorXd::Zero(d_order*(m+1));
      dfp.segment(0, 2*d_order+m-1) = df;
      dfp.segment(2*d_order+m-1, (d_order-1)*(m-1)) = best_dp;
      // get the minimum J
      // VectorXd min_J = dfp.transpose() * R * dfp;
      // get the best coefficients
      VectorXd coeffs = M.inverse() * Ct * dfp;
      //std::cout << "    cal coeffs of one axis done!  size: " << coeffs.rows() << '*' << coeffs.cols() << endl;
      
      // 当前维度的系数 填入
      for(int j = 0; j < m; j ++){
          PolyCoeff.block(j, idx*p_num1d, 1, p_num1d) = coeffs.segment(j*p_num1d, p_num1d).transpose();
          //std::cout << PolyCoeff.block(j, idx*p_num1d, 1, p_num1d) << endl;
      }
  }

  std::cout << "  PolyQPGeneration done, get PolyCoeff! " << endl;
  return PolyCoeff;
}

PolynomialTraj PolynomialTraj::minSnapTraj(const Eigen::MatrixXd &Pos, const Eigen::Vector3d &start_vel,
                                           const Eigen::Vector3d &end_vel, const Eigen::Vector3d &start_acc,
                                           const Eigen::Vector3d &end_acc, const Eigen::VectorXd &Time)
{ 
  int seg_num = Time.size();
  Eigen::MatrixXd poly_coeff(seg_num, 3 * 6);
  Eigen::VectorXd Px(6 * seg_num), Py(6 * seg_num), Pz(6 * seg_num);

  int num_f, num_p; // number of fixed and free variables
  int num_d;        // number of all segments' derivatives

  const static auto Factorial = [](int x) {
    int fac = 1;
    for (int i = x; i > 0; i--)
      fac = fac * i;
    return fac;
  };

  /* ---------- end point derivative ---------- */
  Eigen::VectorXd Dx = Eigen::VectorXd::Zero(seg_num * 6);
  Eigen::VectorXd Dy = Eigen::VectorXd::Zero(seg_num * 6);
  Eigen::VectorXd Dz = Eigen::VectorXd::Zero(seg_num * 6);

  for (int k = 0; k < seg_num; k++)
  {
    /* position to derivative */
    Dx(k * 6) = Pos(0, k);
    Dx(k * 6 + 1) = Pos(0, k + 1);
    Dy(k * 6) = Pos(1, k);
    Dy(k * 6 + 1) = Pos(1, k + 1);
    Dz(k * 6) = Pos(2, k);
    Dz(k * 6 + 1) = Pos(2, k + 1);

    if (k == 0)
    {
      Dx(k * 6 + 2) = start_vel(0);
      Dy(k * 6 + 2) = start_vel(1);
      Dz(k * 6 + 2) = start_vel(2);

      Dx(k * 6 + 4) = start_acc(0);
      Dy(k * 6 + 4) = start_acc(1);
      Dz(k * 6 + 4) = start_acc(2);
    }
    else if (k == seg_num - 1)
    {
      Dx(k * 6 + 3) = end_vel(0);
      Dy(k * 6 + 3) = end_vel(1);
      Dz(k * 6 + 3) = end_vel(2);

      Dx(k * 6 + 5) = end_acc(0);
      Dy(k * 6 + 5) = end_acc(1);
      Dz(k * 6 + 5) = end_acc(2);
    }
  }

  /* ---------- Mapping Matrix A ---------- */
  Eigen::MatrixXd Ab;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  for (int k = 0; k < seg_num; k++)
  {
    Ab = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 0; i < 3; i++)
    {
      Ab(2 * i, i) = Factorial(i);
      for (int j = i; j < 6; j++)
        Ab(2 * i + 1, j) = Factorial(j) / Factorial(j - i) * pow(Time(k), j - i);
    }
    A.block(k * 6, k * 6, 6, 6) = Ab;
  }

  /* ---------- Produce Selection Matrix C' ---------- */
  Eigen::MatrixXd Ct, C;

  num_f = 2 * seg_num + 4; // 3 + 3 + (seg_num - 1) * 2 = 2m + 4
  num_p = 2 * seg_num - 2; //(seg_num - 1) * 2 = 2m - 2
  num_d = 6 * seg_num;
  Ct = Eigen::MatrixXd::Zero(num_d, num_f + num_p);
  Ct(0, 0) = 1;
  Ct(2, 1) = 1;
  Ct(4, 2) = 1; // stack the start point
  Ct(1, 3) = 1;
  Ct(3, 2 * seg_num + 4) = 1;
  Ct(5, 2 * seg_num + 5) = 1;

  Ct(6 * (seg_num - 1) + 0, 2 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 1, 2 * seg_num + 1) = 1; // Stack the end point
  Ct(6 * (seg_num - 1) + 2, 4 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 3, 2 * seg_num + 2) = 1; // Stack the end point
  Ct(6 * (seg_num - 1) + 4, 4 * seg_num + 1) = 1;
  Ct(6 * (seg_num - 1) + 5, 2 * seg_num + 3) = 1; // Stack the end point

  for (int j = 2; j < seg_num; j++)
  {
    Ct(6 * (j - 1) + 0, 2 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 1, 2 + 2 * (j - 1) + 1) = 1;
    Ct(6 * (j - 1) + 2, 2 * seg_num + 4 + 2 * (j - 2) + 0) = 1;
    Ct(6 * (j - 1) + 3, 2 * seg_num + 4 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 4, 2 * seg_num + 4 + 2 * (j - 2) + 1) = 1;
    Ct(6 * (j - 1) + 5, 2 * seg_num + 4 + 2 * (j - 1) + 1) = 1;
  }

  C = Ct.transpose();

  Eigen::VectorXd Dx1 = C * Dx;
  Eigen::VectorXd Dy1 = C * Dy;
  Eigen::VectorXd Dz1 = C * Dz;

  /* ---------- minimum snap matrix ---------- */
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  for (int k = 0; k < seg_num; k++)
  {
    for (int i = 3; i < 6; i++)
    {
      for (int j = 3; j < 6; j++)
      {
        Q(k * 6 + i, k * 6 + j) =
            i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5) * pow(Time(k), (i + j - 5));
      }
    }
  }

  /* ---------- R matrix ---------- */
  Eigen::MatrixXd R = C * A.transpose().inverse() * Q * A.inverse() * Ct;

  Eigen::VectorXd Dxf(2 * seg_num + 4), Dyf(2 * seg_num + 4), Dzf(2 * seg_num + 4);

  Dxf = Dx1.segment(0, 2 * seg_num + 4);
  Dyf = Dy1.segment(0, 2 * seg_num + 4);
  Dzf = Dz1.segment(0, 2 * seg_num + 4);

  Eigen::MatrixXd Rff(2 * seg_num + 4, 2 * seg_num + 4);
  Eigen::MatrixXd Rfp(2 * seg_num + 4, 2 * seg_num - 2);
  Eigen::MatrixXd Rpf(2 * seg_num - 2, 2 * seg_num + 4);
  Eigen::MatrixXd Rpp(2 * seg_num - 2, 2 * seg_num - 2);

  Rff = R.block(0, 0, 2 * seg_num + 4, 2 * seg_num + 4);
  Rfp = R.block(0, 2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2);
  Rpf = R.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 2 * seg_num + 4);
  Rpp = R.block(2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2, 2 * seg_num - 2);

  /* ---------- close form solution ---------- */

  Eigen::VectorXd Dxp(2 * seg_num - 2), Dyp(2 * seg_num - 2), Dzp(2 * seg_num - 2);
  Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf;
  Dyp = -(Rpp.inverse() * Rfp.transpose()) * Dyf;
  Dzp = -(Rpp.inverse() * Rfp.transpose()) * Dzf;

  Dx1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dxp;
  Dy1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dyp;
  Dz1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dzp;

  Px = (A.inverse() * Ct) * Dx1;
  Py = (A.inverse() * Ct) * Dy1;
  Pz = (A.inverse() * Ct) * Dz1;

  for (int i = 0; i < seg_num; i++)
  {
    poly_coeff.block(i, 0, 1, 6) = Px.segment(i * 6, 6).transpose();
    poly_coeff.block(i, 6, 1, 6) = Py.segment(i * 6, 6).transpose();
    poly_coeff.block(i, 12, 1, 6) = Pz.segment(i * 6, 6).transpose();
  }

// // use my method
//   Eigen::MatrixXd poly_coeff = PolyQPGeneration(4, Pos, start_vel, end_vel, start_acc, end_acc, Time);
// // my method over

  /* ---------- use polynomials ---------- */
  PolynomialTraj poly_traj;
  for (int i = 0; i < poly_coeff.rows(); ++i)
  {
    vector<double> cx(6), cy(6), cz(6);
    for (int j = 0; j < 6; ++j)
    {
      cx[j] = poly_coeff(i, j), cy[j] = poly_coeff(i, j + 6), cz[j] = poly_coeff(i, j + 12);
    }
    reverse(cx.begin(), cx.end());
    reverse(cy.begin(), cy.end());
    reverse(cz.begin(), cz.end());
    double ts = Time(i);
    poly_traj.addSegment(cx, cy, cz, ts);
  }

  return poly_traj;
}

PolynomialTraj PolynomialTraj::one_segment_traj_gen(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                    const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
                                                    double t)
{
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Crow(1, 6);
  Eigen::VectorXd Bx(6), By(6), Bz(6);

  C(0, 5) = 1;
  C(1, 4) = 1;
  C(2, 3) = 2;
  Crow << pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1;
  C.row(3) = Crow;
  Crow << 5 * pow(t, 4), 4 * pow(t, 3), 3 * pow(t, 2), 2 * t, 1, 0;
  C.row(4) = Crow;
  Crow << 20 * pow(t, 3), 12 * pow(t, 2), 6 * t, 2, 0, 0;
  C.row(5) = Crow;

  Bx << start_pt(0), start_vel(0), start_acc(0), end_pt(0), end_vel(0), end_acc(0);
  By << start_pt(1), start_vel(1), start_acc(1), end_pt(1), end_vel(1), end_acc(1);
  Bz << start_pt(2), start_vel(2), start_acc(2), end_pt(2), end_vel(2), end_acc(2);

  Eigen::VectorXd Cofx = C.colPivHouseholderQr().solve(Bx);
  Eigen::VectorXd Cofy = C.colPivHouseholderQr().solve(By);
  Eigen::VectorXd Cofz = C.colPivHouseholderQr().solve(Bz);

  vector<double> cx(6), cy(6), cz(6);
  for (int i = 0; i < 6; i++)
  {
    cx[i] = Cofx(i);
    cy[i] = Cofy(i);
    cz[i] = Cofz(i);
  }

  PolynomialTraj poly_traj;
  poly_traj.addSegment(cx, cy, cz, t);

  return poly_traj;
}
