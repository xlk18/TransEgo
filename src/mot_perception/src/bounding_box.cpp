#include "../include/bounding_box.h"

Eigen::VectorXd BoundingBox::toMeasurement() const
{
    Eigen::VectorXd z(6);
    z << position.x(), position.y(), position.z(), dimensions.x(), dimensions.y(), dimensions.z();
    return z;
}
