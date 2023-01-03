#include "main_stream.h"
#include "service/Animateservice.h"
#include "service/SimDynamicsservice.h"
#include "JOY_cardActivator.h"
main_stream::main_stream(QObject *parent) : QObject(parent),paint(NULL)
{
    paint=new MYpainter;
    m_sim=new Simulator;
    m_sim->init();
    Animateservice *Anim_service=JOY_cardActivator::getService<Animateservice>("Animateservice");
    SimDynamicsservice *ODE_service=JOY_cardActivator::getService<SimDynamicsservice>("SimDynamicsservice");


    setupEvent();
    m_sim->update_obs();

    Anim_service->register_painter(paint);
    Anim_service->resizeWindow(1200,800);
    ODE_service->set_ODE(m_sim);
    //这里开始主流程
    Anim_service->start_animate();
    ODE_service->start_sim();


}

void main_stream::setupEvent()
{
    qRegisterMetaType<QMap<int,Single_agent*>>("QMap<int,Single_agent*>");
    connect(m_sim,SIGNAL(sig_agent_state_update(QMap<int,Single_agent*>)),paint,SLOT(slot_update_state(QMap<int,Single_agent*>)),Qt::QueuedConnection);


    qRegisterMetaType<QMap<int,single_obstacle*> >("QMap<int,single_obstacle*>");
    connect(m_sim,SIGNAL(sig_obs_state_update(QMap<int,single_obstacle*>)),paint,SLOT(slot_obs_state_update(QMap<int,single_obstacle*>)),Qt::QueuedConnection);

}
