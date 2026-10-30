// Copyright 2024 realman-robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RM_CONTROL__CUBICSPLINE_H_
#define RM_CONTROL__CUBICSPLINE_H_
// #define RAD_DEGREE       57.2958

class cubicSpline
{
public:
  typedef enum _BoundType
  {
    BoundType_First_Derivative,
    BoundType_Second_Derivative
  } BoundType;

public:
  cubicSpline();
  ~cubicSpline();

  void initParam();
  void releaseMem();

  bool loadData(
    double * x_data, double * y_data, int count, double bound1, double bound2,
    BoundType type);
  bool getYbyX(double & x_in, double & y_out);

protected:
  bool spline(BoundType type);

protected:
  double * x_sample_, * y_sample_;
  double * M_;
  int sample_count_;
  double bound1_, bound2_;
};

#endif  // RM_CONTROL__CUBICSPLINE_H_
