#include "simulator_manager.h"
#include "SwarmSimulatorActivator.h"
#include "algorithm"
simulator_manager::simulator_manager(QObject *parent) : QObject(parent)
{
    Animateservice *Anim_service_0=SwarmSimulatorActivator::getService<Animateservice>("Animateservice");
    SimDynamicsservice *SimDy_service_0=SwarmSimulatorActivator::getService<SimDynamicsservice>("SimDynamicsservice");
    Colli_service=SwarmSimulatorActivator::getService<CollisionDetectservice>("CollisionDetectservice");
    Anim_service=Anim_service_0->cloneservice();
    SimDy_service=SimDy_service_0->cloneservice();

    paint=new MYpainter;
    Anim_service->register_painter(paint);
    m_sim=new Simulator;

    qRegisterMetaType<QMap<int,SwarmAgent*>>("QMap<int,SwarmAgent*>");
    connect(m_sim,SIGNAL(sig_agent_state_update(QMap<int,SwarmAgent*>)),paint,SLOT(slot_update_state(QMap<int,SwarmAgent*>)),Qt::QueuedConnection);


    qRegisterMetaType<QMap<int,SwarmObstacle*> >("QMap<int,SwarmObstacle*>");
    connect(m_sim,SIGNAL(sig_obs_state_update(QMap<int,SwarmObstacle*>)),paint,SLOT(slot_obs_state_update(QMap<int,SwarmObstacle*>)),Qt::QueuedConnection);
    m_sim->init();
}

Animateservice *simulator_manager::getwidgetoperator()
{
    return Anim_service;
}

SwarmSimservice *simulator_manager::cloneservice()
{
    simulator_manager* new_service=new simulator_manager;
    return new_service;
}

SimDynamicsservice *simulator_manager::get_simer()
{
    return SimDy_service;
}

QMap<int, SwarmAgent *> simulator_manager::getagentgroup()
{
    return m_sim->Agents_group;
}

void simulator_manager::startsim()
{
    SimDy_service->set_delta_t(m_sim->delta_t);
    SimDy_service->set_ODE(m_sim);
    SimDy_service->start_sim();
}

void simulator_manager::init_plant(int steptime, int init_method, SwarmAgent *singleagents)
{
    if(init_method==0)
    {
        //按照给定文件初始化
        QString agent_mat=f_Read_TXT("./config/agent_mat.txt");//纵向为agent数量，横向为状态
        QString obs_mat=f_Read_TXT("./config/obs_mat.txt");//每一行为一个obs，xy坐标是级联的
        //f_get_mat_at是支持任意不规则形状的数据组的，也就是允许每列或者每一行不等长
        int agent_num=f_get_mat_height(agent_mat);
        int status_num=f_get_mat_width(agent_mat);
        int obs_num=f_get_mat_height(obs_mat);
        m_sim->delta_t=steptime;
        m_sim->agent_num=agent_num;
        m_sim->obs_num=obs_num;
        //应当使用工厂模式创造obs和agent，所以,agent_creator应该被传入
        for(int i=1;i<=obs_num;i++)
        {
            SwarmObstacle *new_obs=new SwarmObstacle;//这个应该暂时是不需要什么拓展性的
            new_obs->ID=i;
            new_obs->vertex_num=f_get_line_length(obs_mat,i-1)/2;

            std::vector<double> point_x_vec;
            std::vector<double> point_y_vec;

            for(int j=1;j<=new_obs->vertex_num;j=j+2)
            {
                double x= f_get_mat_at(i-1,(j-1)/2,obs_mat).toDouble();
                double y= f_get_mat_at(i-1,1+(j-1)/2,obs_mat).toDouble();
                point_x_vec.push_back(x);
                point_y_vec.push_back(y);
                new_obs->vertex_map<<QPointF(x,y);
            }

            double max_x = *max_element(point_x_vec.begin(), point_x_vec.end());
            double max_y = *max_element(point_y_vec.begin(), point_y_vec.end());
            double min_x = *min_element(point_x_vec.begin(), point_x_vec.end());
            double min_y = *min_element(point_y_vec.begin(), point_y_vec.end());

            new_obs->x_up=max_x+singleagents->collision_r;
            new_obs->x_low=min_x-singleagents->collision_r;
            new_obs->y_up=max_y+singleagents->collision_r;
            new_obs->y_low=min_y-singleagents->collision_r;

            m_sim->obsbounding_group.insert(i,new_obs);
        }

        for(int i=1;i<=agent_num;i++)
        {
            SwarmAgent *new_agent=singleagents->clone_agent();//这个clone会保留一些关键设置参数
            new_agent->state_vector.resize(status_num,1);
            for(int j=0;j<new_agent->status_num;j++)
            {
                new_agent->state_vector(j,0)=f_get_mat_at(i-1,j,agent_mat).toDouble();
                new_agent->act_vector.resize(new_agent->action_num,1);
                new_agent->act_vector.setZero();
            }
            m_sim->Agents_group.insert(i,new_agent);
        }
        m_sim->state_array.resize(agent_num,status_num);
        m_sim->act_array.resize(agent_num,singleagents->action_num);
        m_sim->act_array.setZero();
        m_sim->state_array.setZero();

        m_sim->agent_communication_range=m_sim->Agents_group.value(1)->communication_range;
        m_sim->collision_r=m_sim->Agents_group.value(1)->collision_r;

    }
}
