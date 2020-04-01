#include "workcellcontainer.h"

#include <rw/loaders/WorkCellLoader.hpp>

#include <rw/common/Ptr.hpp>

WorkCellContainer::WorkCellContainer(std::string wcFileName)
{
    std::cout << "Loading workcell from: " << wcFileName << std::endl;
    workcell = rw::loaders::WorkCellLoader::Factory::load(wcFileName);
    if (workcell.isNull())
        throw std::runtime_error("Could not load workcell form: " + wcFileName);

    std::unique_lock<std::mutex> lock(stateMtx);
    state = workcell->getDefaultState();
}
