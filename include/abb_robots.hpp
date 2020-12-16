#pragma once
#include <list>



namespace abb_robots {

// CLASSES ===============================================================================
class Robot {
  private:
    struct AxisRange {
        const int min;
        const int max;
        AxisRange(const int min_, const int max_) : min(min_), max(max_){};
        AxisRange(const AxisRange &axis) : min(axis.min), max(axis.max){};
    };
    struct RobotLimits {
        const AxisRange axis1;
        const AxisRange axis2;
        const AxisRange axis3;
        const AxisRange axis4;
        const AxisRange axis5;
        const AxisRange axis6;
        RobotLimits(const AxisRange axis1_, const AxisRange &axis2_,
                    const AxisRange &axis3_, const AxisRange &axis4_,
                    const AxisRange &axis5_, const AxisRange &axis6_)
            : axis1(axis1_), axis2(axis2_), axis3(axis3_), axis4(axis4_), axis5(axis5_),
              axis6(axis6_){};
    };

  public:
    const int model;
    const double weight;             // [kg].
    const double handling_capacity;  // [kg].
    const double reach;              // [m].
    const RobotLimits axis;
    Robot(const int model_, const double weight_, const double handling_capacity_,
          const double reach_, const RobotLimits axis_)
        : model(model_), weight(weight_), handling_capacity(handling_capacity_),
          reach(reach_), axis(axis_){};
};



// ROBOT INSTANCES =======================================================================
static Robot IRB_1100{
    1110,   // name.
    21.1,   // weight.
    4.0,    // handling_capacity.
    0.475,  // reach.
    {
        {-230, 230},  // [degrees] axis1.
        {-115, 113},  // [degrees] axis2.
        {-205, 55},   // [degrees] axis3.
        {-230, 230},  // [degrees] axis4.
        {-125, 120},  // [degrees] axis5.
        {-400, 400}   // [degrees] axis6.
    },
};

}  // namespace abb_robots