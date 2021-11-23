#include <vector>

template<typename T>
class Cube {
  std::vector<T> data;
  const size_t dim_x;
  const size_t dim_y;
public:
  Cube(size_t X, size_t Y, size_t Z, T t) : data(X*Y*Z, t), dim_x(X), dim_y(Y) {}
  T operator()(size_t x, size_t y, size_t z) const { return data[z*(dim_y*dim_x) + y*dim_x + x]; }
  T& operator()(size_t x, size_t y, size_t z) { return data[z*(dim_y*dim_x) + y*dim_x + x]; }
};

// z*(DIM_Y*DIM_X) + y*DIM_X + x;