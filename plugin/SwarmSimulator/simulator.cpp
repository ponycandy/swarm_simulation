#include "simulator.h"
#include <QElapsedTimer>
#include <QCoreApplication>
#include "SwarmSimulatorActivator.h"
#include "QtDebug"
Simulator::Simulator(QObject *parent)
{

}
void Simulator::update_obs()
{
    emit sig_obs_state_update(obsbounding_group);

}

double Simulator::get_angle(double vx, double vy)
{
    double value=0;
    if(vx==0 && vy==0)
    {
        return value;
    }
    else
    {
        value=atan2(vy,vx);
        return  value;
    }
}

void Simulator::init()

{
    m_service=SwarmSimulatorActivator::getService<CollisionDetectservice>("CollisionDetectservice");
    update_obs();
}
void Simulator::ode_function(Eigen::MatrixXd act_mat, Eigen::MatrixXd state_mat)
{
    for(int i=1;i<=agent_num;i++)
    {
        SwarmAgent *agent=Agents_group.value(i);
        agent->state_vector=agent->state_vector+0.001*delta_t*agent->state_space_equation();
        agent->state_update();
    }

}

void Simulator::control(Eigen::MatrixXd sensor, Eigen::MatrixXd state_mat)
{
    for(int i=1;i<=agent_num;i++)
    {
        SwarmAgent *agent=Agents_group.value(i);
        agent->controlfunction();
    }
}
void Simulator::sensor(Eigen::MatrixXd state_mat)
{
    detect_neibor(state_array);
    detect_collision();
//系统完成默认服务：碰撞检测和邻居识别，然后进入agent环节
    for(int i=1;i<=agent_num;i++)
    {
        SwarmAgent *agent=Agents_group.value(i);
        agent->sensorfunction();
    }
}

void Simulator::broad_cast(Eigen::MatrixXd state_mat, Eigen::MatrixXd act_mat, Eigen::MatrixXd sensor)
{
    emit sig_agent_state_update(Agents_group);
    for(int i=1;i<=agent_num;i++)
    {
        SwarmAgent *agent=Agents_group.value(i);
        agent->broadcastfunction();
    }
}
void Simulator::detect_neibor(Eigen::MatrixXd state_mat)
{
    double dis=0;
    neib_array.setZero();
    for(int i=1;i<=agent_num;i++)
    {
        Agents_group.value(i)->remove_all_neibors();
    }
    for(int iter1=1;iter1<=agent_num;iter1++)
    {
        for(int iter2=1;iter2<iter1;iter2++)
        {
            dis=pow(Agents_group.value(iter1)->pos_xy(0,0)-Agents_group.value(iter2)->pos_xy(0,0),2)
                    +pow(Agents_group.value(iter1)->pos_xy(1,0)-Agents_group.value(iter2)->pos_xy(1,0),2);
            if(dis<pow(Agents_group.value(iter1)->communication_range,2))
            {
                neib_array(iter1-1,iter2-1)=1;
                neib_array(iter2-1,iter1-1)=1;
                Agents_group.value(iter1)->appendNeibor(Agents_group.value(iter2));
                Agents_group.value(iter2)->appendNeibor(Agents_group.value(iter1));

            }

        }
    }

}
bool Simulator::within_range(Eigen::MatrixXd posxy, SwarmObstacle *obs)
{
    if(posxy(0,0)<obs->x_low ||  posxy(0,0)>obs->x_up || posxy(1,0)<obs->y_low ||  posxy(1,0)>obs->y_up)
    {
        return false;
    }
    else
    {
        return true;
    }
}
void Simulator::detect_collision()
{
    for(int i=1;i<=agent_num;i++)
    {
        Agents_group.value(i)->remove_all_closepoint();
        for(int j=1;j<=obs_num;j++)
        {
            SwarmObstacle *obs=obsbounding_group.value(j);
            if(within_range(Agents_group.value(i)->pos_xy,obs)==1)
            {
                collison_result result=m_service->polygen_circle_detect(Agents_group.value(i)->pos_xy(0,0),
                                    Agents_group.value(i)->pos_xy(1,0),Agents_group.value(i)->collision_r,obs->vertex_map);
                if(result.flag==1)
                {
                    SwarmAgent* as=Agents_group.value(i);
                    result.closest_point.obs_ID=j;
                    ClosePoint *a_point=new ClosePoint;
                    a_point->x=result.closest_point.x;
                    a_point->y=result.closest_point.y;
                    a_point->obs_ID=result.closest_point.obs_ID;
                    a_point->edge1_x=result.closest_point.edge1_x;
                    a_point->edge1_y=result.closest_point.edge1_y;
                    a_point->edge2_x=result.closest_point.edge2_x;
                    a_point->edge2_y=result.closest_point.edge2_y;
                    a_point->next_vertex=result.closest_point.next_vertex;
                    a_point->last_vertex=result.closest_point.last_vertex;
                    a_point->update_state();

                    as->append_closepointpair(j,a_point);
                    //Eigen probelm
                }
            }
            else
            {

            }
        }

    }

}
