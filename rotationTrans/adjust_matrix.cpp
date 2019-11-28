#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <thread>

using namespace std;

Eigen::Matrix3f R_;
Eigen::Vector3f euler_(0, 0, 0);

void CommandProcess()
{
  std::cout << "initial R:\n"
            << R_ << std::endl;

  char buff[128] = {0};
  while (1)
  {
    if (NULL != fgets(buff, 128, stdin))
    {
      char c;
      double value;
      sscanf(buff, "%c%lf", &c, &value);
      switch (c)
      {
      case 'w': // rotation/euler/omega
      {
        char a;
        float d[3];
        sscanf(buff, "%c%f %f %f", &a, &d[0], &d[1], &d[2]);
        euler_ = Eigen::Vector3f(d[0], d[1], d[2]) / 57.3f;
        Eigen::Matrix3f mat;
        mat = Eigen::AngleAxisf(euler_[0], Eigen::Vector3f::UnitX()) *
              Eigen::AngleAxisf(euler_[1], Eigen::Vector3f::UnitY()) *
              Eigen::AngleAxisf(euler_[2], Eigen::Vector3f::UnitZ());
        R_ = mat * R_;
        Eigen::Quaternionf q(R_);
        R_ = q.normalized().toRotationMatrix();

        cout << "new Transform:\n"
             << R_ << endl << endl;

        for(int i=0;i<3;i++)
	  printf("%f, %f, %f,\n", R_(i,0), R_(i,1), R_(i,2));
        break;
      }
      default:
        break;
      }
    }

    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char **argv)
{
  R_<<0.164449,  0.0100039,  0.986335,
         0.0102009,-0.999912,   0.00844087,
         0.986333,  0.00867341, -0.164536;

  cout << "R:\n"
       << R_ << endl;

  std::thread keyboard_command_process = std::thread(CommandProcess);

  keyboard_command_process.join();
}
