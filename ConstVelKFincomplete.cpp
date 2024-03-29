//Incomplete
#include <iostream>
#include <cmath>
#include <array>

template <typename T, size_t rc>
std::array<std::array<T, rc>, rc> diagonal_matrix(const std::array<T, rc>& diagonal_values) {
  std::array<std::array<T, rc>, rc> result{};
  for (int i = 0; i < rc; i++) {
    result[i][i] = diagonal_values[i];
  }
  return result;
}
template <typename T, size_t Rows, size_t Cols>
std::array<std::array<T, Rows>, Cols> transpose(const std::array<std::array<T, Cols>, Rows>& original_matrix) {
  std::array<std::array<T, Rows>, Cols> result{};
  for (int row = 0; row < Rows; row++) {
    for (int col = 0; col < Cols; col++) {
    result[col][row] = original_matrix[row][col];
  }
  }
  return result;
}





class ConstVelKF {
  private:
  std::array<std::array<float, 6>, 6> x;
  std::array<std::array<float, 6>, 6> P;
  int dim;
  int angle_idx;
  public:
  std::array<std::array<float, 6>, 6> Q;
  std::array<std::array<float, 3>, 3> R;
  std::array<float, 6> diagonalQ = {1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3};
  std::array<float, 3> diagonalR = {2.444e-3, 1.2527e-3, 4.0482e-3};
  std::array<float, 6> diagonalone = {1,1,1,1,1,1};
  float MAX_DT;
  
  ConstVelKF(const std::array<std::array<float, 6>, 6> x0, std::array<std::array<float, 6>, 6> P0, int dim  = 3, int angle_idx = 2): x(x0), P(P0), dim(dim), angle_idx(angle_idx)  {
    Q = diagonal_matrix(diagonalQ);
    R = diagonal_matrix(diagonalR);
    MAX_DT = 1e-3;
  }

  void process_update(float dt) {
    if (dt <= 0) {
      return;
    }

    std::array<std::array<float, 6>, 6> A;
    A = diagonal_matrix(diagonalone);
    
    
  }

  double wrap_angle(double theta) {
    return atan2(sin(theta), cos(theta));
  } 




}; 

int main() {
  /*std::cout << "Hello World!\n";
  const int dim = 3;
  int a[dim * 2][dim * 2] = {{0}};
  int arrb[dim * 2][dim * 2] = {{0}};
  int val = 1;
  int row, col;
  for(row=0; row < 6; row++) {

    for(col=0; col< 6; col++) {
      if(row == col) {
        a[row][col] = 2;
      }
    }
  }  
  for(row=0; row < 6; row++) {
    for(col=0; col< 6; col++) {
      if(row == col) {
        arrb[row][col] = val;
        val++;
      }
    }
  }  
  for(row=0; row < 6; row++) {
    for(col=0; col < 6; col++) {
      int index = a[row][col] * arrb[col][row];
      std::cout<<"\t"<<index;
    }
    std::cout<<std::endl;
  }
  double x0[6][6] = {{0}};
  double P0[6][6] = {{0}};
  ConstVelKF cvkf(x0, P0);
  cvkf.printQ();
  cvkf.printR();*/
}
