#ifndef MATRIX_HPP_
#define MATRIX_HPP_

#include <cmath>
#include <vector>
#include <string>
#include <cassert>
#include <sstream>

namespace Model{
  class Matrix
  {
    public:
      Matrix(size_t width, size_t height, const std::vector<double>& data)
      : data(data)
      , width(width)
      , height(height)
      {}
      
      Matrix(size_t width, size_t height)
      : data(width*height)
      , width(width)
      , height(height)
      {}

      Matrix(const std::vector<double>& data)
      : data(data)
      , width(1)
      , height(data.size())
      {}

      static Matrix identity(size_t n, double value = 1.0f){
        Matrix result(n,n);
        for(size_t i = 0; i < n; ++i){
          result.at(i,i) = value;
        }
        return result;
      }

      Matrix operator=(const Matrix& rhs)
      {
        if(this == &rhs) return *this;
        width = rhs.width;
        height = rhs.height;
        data = rhs.data;
        return *this;
      }

      Matrix operator=(double newValue)
      {
        for(auto& value : data){
          value = newValue;
        }
        return *this;
      }

      Matrix operator+(const Matrix& rhs) const
      {
        assert(width == rhs.width && height == rhs.height && "Matrix::operator+ matrix sizes must match");
        Matrix result(width,height);
        for(size_t i = 0; i < width*height; ++i)
        {
          result[i] = data[i] + rhs[i];
        }
        return result;
      }
      
      Matrix operator+(double rhs) const
      {
        Matrix result(width,height);
        for(size_t i = 0; i < width*height; ++i)
        {
          result[i] = data[i] + rhs;
        }
        return result;
      }
      
      Matrix operator+=(const Matrix& rhs)
      {
        assert(width == rhs.width && height == rhs.height && "Matrix::operator+ matrix sizes must match");
        for(size_t i = 0; i < width*height; ++i)
        {
          data[i] += rhs[i];
        }
        return *this;
      }
      
      Matrix operator-(const Matrix& rhs) const
      {
        assert(width == rhs.width && height == rhs.height && "Matrix::operator- matrix sizes must match");
        Matrix result(width,height);
        for(size_t i = 0; i < width*height; ++i)
        {
          result[i] = data[i] - rhs[i];
        }
        return result;
      }
      
      Matrix operator-=(const Matrix& rhs)
      {
        assert(width == rhs.width && height == rhs.height && "Matrix::operator- matrix sizes must match");
        for(size_t i = 0; i < width*height; ++i)
        {
          data[i] -= rhs[i];
        }
        return *this;
      }
      
      Matrix operator*(double rhs) const
      {
        Matrix result(width,height);
        for(size_t i = 0; i < width*height; ++i)
        {
          result[i] = data[i] * rhs;
        }
        return result;
      }

      Matrix operator/(double rhs) const
      {
        Matrix result(width,height);
        for(size_t i = 0; i < width*height; ++i)
        {
          result[i] = data[i] / rhs;
        }
        return result;
      }

      Matrix dot(const Matrix& rhs)
      {
        assert(rhs.height == width && "lhs.width must equal rhs.height");
        Matrix result(rhs.width,height);
        for(size_t x = 0; x < rhs.width; x++){
          for(size_t y = 0; y < height; y++){
            double acc = 0.0f;
            for(size_t i = 0; i < width; ++i){
              acc += at(i,y) * rhs.at(x,i);
            }
            result.set(x,y,acc);
          }
        }
        return result;
      }

      Matrix transpose() const
      {
        Matrix result(height,width);
        for(size_t x = 0; x < width; x++){
          for(size_t y = 0; y < height; y++){
            result.at(x,y) = at(y,x);
          }
        }
        return result;
      }

      Matrix cofactor(size_t row, size_t col) const
      {
        assert(width > 1 &&       "Matrix::cofactor; cannot calculate cofactor of matrix less than 2");
        assert(width == height && "Matrix::cofactor: can only calculate cofactor of square matrix");
        size_t i = 0;
        size_t j = 0;
        Matrix result(width-1,height-1);
        for(size_t r = 0; r < width; ++r){
          for(size_t c = 0; c < width; ++c){
            if(r != row && c != col){
              result.at(i,j) = at(r,c);
              j++;
            }
            if(j >= width - 1){
              j = 0;
              i++;
            }
          }
        }
        return result;
      }

      double determinant() const
      {
        assert(width > 0 &&       "Matrix::determinant; cannot calculate determinant of nothing");
        assert(width == height && "Matrix::determinant: can only calculate determinant of square matrix");

        if(width == 1){
          return data[0];
        }else if( width == 2){
          return (at(0,0) * at(1,1)) - (at(0,1) * at(1,0));
        }

        double det = 0.0f;
        double sign = 1.0f;
        for(size_t i = 0; i < width; ++i){
          det += sign * at(0,i) * cofactor(0,i).determinant();
          sign = -sign;
        }
        return det;
      }

      Matrix adjoint() const
      {
        assert(width > 0 &&       "Matrix::adjoint; cannot calculate adjoint of nothing");
        assert(width == height && "Matrix::adjoint: can only calculate adjoint of square matrix");
        if(width == 1){
          return Matrix(1,1,{1});
        }
        
        Matrix result(width,height);
        double sign = 1.0f;
        for(size_t i = 0; i < width; ++i){
          for(size_t j = 0; j < width; ++j){
            sign = ((i+j)%2 == 0)? 1.0f : -1.0f;
            
            // note reversed rows and columns because need the transpose
            result.at(j,i) = sign * cofactor(i,j).determinant();
          }
        }
        return result;
      }

      
      Matrix inverse() const
      {
        assert(width > 0 &&       "Matrix::inverse; cannot calculate inverse of nothing");
        assert(width == height && "Matrix::inverse: can only calculate inverse of square matrix");
        
        double det = determinant();
        assert(det != 0.0f &&     "Matrix::inverse: cannot calculate inverse of matrix with determinant of 0");

        double d = 1.0/determinant();
        Matrix result(width,height);
        Matrix adjoint(this->adjoint());
        
        for(size_t i = 0; i < width; ++i){
          for(size_t j = 0; j < width; ++j){
            result.at(i,j) = adjoint.at(i,j)/det;
          }
        }
        return result;
      }

      double at(size_t x, size_t y) const
      {
        return data[x+y*width];
      }
      
      double& at(size_t x, size_t y)
      {
        return data[x+y*width];
      }

      double& operator[](size_t i){
        return data[i];
      }
      
      double operator[](size_t i) const{
        return data[i];
      }

      void set(size_t x, size_t y, double val)
      {
        data[x+y*width] = val;
      }

      size_t size() const
      {
        return data.size();
      }

      size_t getWidth() const
      {
        return width;
      }

      size_t getHeight() const
      {
        return height;
      }

      std::string asDebugString() const
      {
        std::ostringstream oss;
        oss << this;
        return "Matrix( "+std::to_string(width)+", "+std::to_string(height)+") at "+oss.str();  
      }
    private:
      std::vector<double> data;
      size_t width;
      size_t height;
  };
} // namespace Model
#endif // MATRIX_HPP_
