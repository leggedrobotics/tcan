\page page_average_calculator Average Calculator

# Average Calculator

## Overview

This package contains tools to calculate the average of given data points.

## Installation

### Install Debian Packages

To install all packages from the this repository as Debian packages use

    sudo apt install ros-${ROS_DISTRO}-...

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

    cd ~/catkin_ws/src
    git clone git@code.anymal.com:anymal-research/anymal_research.git
    catkin build average_calculator

### Unit Tests

Run the unit tests with

    catkin run_tests average_calculator

## Usage

Example code computing the average of given values:

```
// average calculator
#include <average_calculator/AverageCalculator.hpp>

using namespace average_calculator;

int main(int argc, char** argv)
{
  AverageCalculator averageCalculator;
  averageCalculator.addValue(1.0);
  averageCalculator.addValue(3.0);
  averageCalculator.addValue(-2.0);
  const double averageValue = averageCalculator.getAverageValue();
  return 0;
}
```
