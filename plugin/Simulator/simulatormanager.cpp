#include "simulatormanager.h"
#include "SimulatorActivator.h"
SimulatorManager::SimulatorManager(QObject *parent) : QObject(parent)
{
    delt_t=30;//ms
    m_sim=new Simer(delt_t);
    SimulatorActivator::registerservice(this,"SimDynamicsservice");
}

void SimulatorManager::set_delta_t(double delta_t)
{
    delt_t=delta_t;
    m_sim->dt=delt_t;
}

void SimulatorManager::start_sim()
{
    m_sim->run();
}

void SimulatorManager::set_ODE(ODE_functor *functor)
{
    m_sim->functor=functor;
}
