#ifndef WORKCELLCONTAINER_H
#define WORKCELLCONTAINER_H

#include <string>
#include <mutex>

#include <rw/models/WorkCell.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/kinematics/State.hpp>



class WorkCellContainer
{
public:
    WorkCellContainer(std::string wcFileName);

public:
    rw::kinematics::State state;
    std::mutex stateMtx;
    rw::models::WorkCell::Ptr workcell;



};

#endif // WORKCELLCONTAINER_H
