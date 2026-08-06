#include <bspline_race/UniformBspline_3d.h>

#include <cmath>
#include <limits>

namespace FLAG_Race

{
    UniformBspline::UniformBspline(const int &p,  const int &n, const double &beta, const int &D, 
                                                 const Eigen::MatrixXd &s_ini, const Eigen::MatrixXd &s_ter)
    {
        initUniformBspline(p, n,beta, D, s_ini, s_ter); 
    }

    UniformBspline::~UniformBspline() {}

    void UniformBspline::init(ros::NodeHandle& nh)
    {
        nh.param("planning/traj_order", p_, 3);
        nh.param("planning/dimension", D_, -1);
        nh.param("planning/dist_p",dist_p,0.5);
        nh.param("planning/max_vel", max_vel_, -1.0);
        nh.param("planning/TrajSampleRate", TrajSampleRate, 1);
        
        beta_ = max_vel_/dist_p;

        std::cout << "\033[1;32m" << "success init Bspline module" << "\033[0m" << std::endl;
        
    }

    void UniformBspline::initUniformBspline(const int &p,  const int &n, const double &beta, const int &D, 
                                                 const Eigen::MatrixXd &s_ini, const Eigen::MatrixXd &s_ter)
    {
        p_ = p; 
        n_ = n-1;
        beta_ = beta;
        D_ =D;
        m_ = p_+n_+1;
        u_ = Eigen::VectorXd::Zero(m_ + 1); //u0 ~ um 共m+1个
        control_points_ = Eigen::MatrixXd::Zero(n_+1,D_);
        for(int i = 0; i<=m_; i++)
        {
            u_(i) = i;
        }
        s_ini_ = s_ini;
        s_ter_ = s_ter;
        setIniTerMatrix();
        getAvailableSrange();
        getAvailableTrange();
        getInterval();
    }

    bool UniformBspline::parameterizeToBspline(
        double ts,
        const std::vector<Eigen::Vector3d>& point_set,
        const std::vector<Eigen::Vector3d>& start_end_derivatives,
        Eigen::MatrixXd& control_points) {
      if (!std::isfinite(ts) || ts <= 0.0 || point_set.size() < 2 ||
          start_end_derivatives.size() != 4) {
        return false;
      }

      const int k = static_cast<int>(point_set.size());
      Eigen::MatrixXd a = Eigen::MatrixXd::Zero(k + 4, k + 2);
      const Eigen::RowVector3d position_row(1.0, 4.0, 1.0);
      const Eigen::RowVector3d velocity_row(-1.0, 0.0, 1.0);
      const Eigen::RowVector3d acceleration_row(1.0, -2.0, 1.0);

      for (int i = 0; i < k; ++i) {
        a.block<1, 3>(i, i) = position_row / 6.0;
      }
      a.block<1, 3>(k, 0) = velocity_row / (2.0 * ts);
      a.block<1, 3>(k + 1, k - 1) = velocity_row / (2.0 * ts);
      a.block<1, 3>(k + 2, 0) = acceleration_row / (ts * ts);
      a.block<1, 3>(k + 3, k - 1) = acceleration_row / (ts * ts);

      Eigen::MatrixXd b(k + 4, 3);
      for (int i = 0; i < k; ++i) b.row(i) = point_set[i].transpose();
      for (int i = 0; i < 4; ++i) {
        b.row(k + i) = start_end_derivatives[i].transpose();
      }

      control_points = a.colPivHouseholderQr().solve(b);
      return control_points.rows() == k + 2 && control_points.cols() == 3 &&
             control_points.allFinite();
    }

    bool UniformBspline::setControlPointsAndInterval(
        const Eigen::MatrixXd& control_points, int order, double interval) {
      if (order < 1 || control_points.cols() != 3 ||
          control_points.rows() < order + 1 || !control_points.allFinite() ||
          !std::isfinite(interval) || interval <= 0.0) {
        return false;
      }

      p_ = order;
      D_ = static_cast<int>(control_points.cols());
      n_ = static_cast<int>(control_points.rows()) - 1;
      m_ = p_ + n_ + 1;
      beta_ = 1.0 / interval;
      control_points_ = control_points;
      u_ = Eigen::VectorXd::Zero(m_ + 1);
      for (int i = 0; i <= m_; ++i) u_(i) = static_cast<double>(i);
      setIniTerMatrix();
      getAvailableSrange();
      getAvailableTrange();
      getInterval();
      time_.resize(0);
      return true;
    }

    double UniformBspline::getFeasibilityRatio(double max_vel, double max_acc) const {
      if (!std::isfinite(max_vel) || !std::isfinite(max_acc) ||
          max_vel <= 0.0 || max_acc <= 0.0 || control_points_.rows() < 2) {
        return std::numeric_limits<double>::infinity();
      }

      double velocity_max = 0.0;
      for (int i = 0; i + 1 < control_points_.rows(); ++i) {
        const Eigen::RowVectorXd velocity =
            beta_ * (control_points_.row(i + 1) - control_points_.row(i));
        velocity_max = std::max(velocity_max, velocity.cwiseAbs().maxCoeff());
      }

      double acceleration_max = 0.0;
      for (int i = 0; i + 2 < control_points_.rows(); ++i) {
        const Eigen::RowVectorXd acceleration = beta_ * beta_ *
            (control_points_.row(i + 2) - 2.0 * control_points_.row(i + 1) +
             control_points_.row(i));
        acceleration_max =
            std::max(acceleration_max, acceleration.cwiseAbs().maxCoeff());
      }

      return std::max(velocity_max / max_vel,
                      std::sqrt(acceleration_max / max_acc));
    }

    bool UniformBspline::scaleTime(double ratio) {
      if (!std::isfinite(ratio) || ratio <= 0.0 ||
          !std::isfinite(beta_) || beta_ <= 0.0) {
        return false;
      }
      beta_ /= ratio;
      setIniTerMatrix();
      getAvailableTrange();
      getInterval();
      time_.resize(0);
      return true;
    }

    void UniformBspline::setIniTerMatrix()
    {
        A_ini.resize(3,3);
        A_ter.resize(3,3);
        A_ini << 1.0/6, 2.0/3, 1.0/6,
                        -1.0/2*beta_, 0.0*beta_, 1.0/2*beta_,
                        1.0*beta_*beta_,-2.0*beta_*beta_,1.0*beta_*beta_;
        A_ter<<1.0/6, 2.0/3, 1.0/6,
                        -1.0/2*beta_, 0.0*beta_, 1.0/2*beta_,
                        1.0*beta_*beta_,-2.0*beta_*beta_,1.0*beta_*beta_;
    }

    void UniformBspline::setControlPoints(const Eigen::MatrixXd &ctrl_points)
    {
        control_points_ = ctrl_points;
    }
    
    Eigen::MatrixXd UniformBspline::getTrajectory(const Eigen::VectorXd &t)
    {
        double u_probe;
        int t_size = t.size();
        Eigen::MatrixXd trajectory(t_size,D_);
        for (size_t i = 0; i < t_size; i++)
        {
            //map t(i) to uniform knot vector
            u_probe = t(i) * beta_ + u_(p_);
            trajectory.row(i) = singleDeboor(u_probe); 
        }
        return trajectory;
    }

     Eigen::Vector3d UniformBspline::singleDeboor(const double &u_probe) const//the deboor's algorithm
     {  
        //bound the u_probe
        double u_probe_;
        int k;
        u_probe_ = min(max( u_(p_) , u_probe), u_(m_-p_));
        k = p_;
        while(true)
        {
            if(u_(k+1)>=u_probe_)
                break;
            k = k+1;
        }
        // t(t_ctt) is maped to knot segment u_k ~ u_{k+1}
        // there are at most p+1 basis functions N_k-p,p(u), N_k-p+1,p(u),..., N_k,p(u) non-zero on knot span [uk,uk+1)
        //the effective control points are P_k-p ~ P_k
        // since MATLAB index start from 1 instead of 0
        // The effective control points are
        double alpha;
        Eigen::MatrixXd d(p_+1,3);
        d = control_points_.block(k-p_,0,p_+1,3);// c++这里是从0行0列开始
        for (size_t i = 0; i < p_; i++)
        {
            for (size_t j = p_; j > i; j--)
            {
                alpha = (u_probe_ - u_(j+k-p_)) /(u_(j+k-i) - u_(j+k-p_)); 
                d.row(j) = (1 - alpha)*d.row(j-1) + alpha*d.row(j);
            }          
        }

            Eigen::Vector3d value;
            value = d.row(p_);
            return value;
     }

    void UniformBspline::getAvailableSrange()
    {
        s_range = {u_(p_),u_(m_-p_)};
    }

    void UniformBspline::getAvailableTrange()
    {
        t_range = {0/beta_, (u_(m_-p_)-u_(p_))/beta_};
    }

    void UniformBspline::getInterval()
    {
        interval_ = (u_(1) - u_(0))/beta_;
    }

    void UniformBspline::getT()
    {
        int trajSampleRate = TrajSampleRate;
        time_.resize((t_range(1)-t_range(0))*trajSampleRate+1);
        
        for (size_t i = 0; i < time_.size(); i++)
        {
            time_(i) = t_range(0) + i*(1.0/trajSampleRate);
        }
    }

    UniformBspline UniformBspline::getDerivative() const
    {     
            UniformBspline spline(p_,n_,beta_,D_,s_ini_,s_ter_);
            spline.p_ = spline.p_ -1;
            spline.m_ = spline.p_ +spline.n_ +1;
            spline.u_.resize(u_.size()-2);
            spline.u_ = u_.segment(1,m_-1);//从第2个元素开始的m-1个元素
            spline.control_points_.resize(control_points_.rows()-1,D_);
            for (size_t i = 0; i < spline.control_points_.rows(); i++)
            {
                spline.control_points_.row(i) = spline.beta_*(control_points_.row(i+1) - control_points_.row(i));
            } 
            spline.time_ = time_;
            return spline;
    }

    Eigen::VectorXd UniformBspline::getBoundConstraintb()
    {
        int nm = (n_+1)*D_;
        Eigen::VectorXd b= Eigen::VectorXd::Zero(nm);
        Eigen::MatrixXd tmp1(3,D_);//前三个控制点的值
        Eigen::MatrixXd tmp2(3,D_);//末尾三个控制点的值
        // solve Ax = b
        tmp1 = A_ini.colPivHouseholderQr().solve(s_ini_);
        tmp2 = A_ter.colPivHouseholderQr().solve(s_ter_);
         for (size_t j = 0; j< D_; j++)
        {
            for (size_t i = 0; i < 3; i++)
            {
                b(i+j*(n_+1)) = tmp1(i,j);
                b((j+1)*(n_+1)-i-1) = tmp2(3-i-1,j);
            }      
        }    
        return b;   
    }

}
