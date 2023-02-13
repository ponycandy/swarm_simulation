#include "SwarmAgent.h"

SwarmAgent::SwarmAgent()
{

    pos_xy.resize(2,1);
    pos_xy.setZero();
    vel_xy.resize(2,1);
    vel_xy.setZero();
    neib_num=0;
    obs_closet_point_num=0;
}

void SwarmAgent::appendNeibor(SwarmAgent *agent)
{
    neib_num+=1;
    neib_map.insert(neib_num,agent);  //一般按照从近到远的顺序append
}

void SwarmAgent::remove_all_closepoint()
{
    for(int it=1;it<=obs_closet_point_num;it++)
    {
        closepoint_map.remove(it);
    }
    obs_closet_point_num=0;
}

void SwarmAgent::remove_all_neibors()
{
    for(int it=1;it<=neib_num;it++)
    {
        neib_map.remove(it);
    }
    neib_num=0;
}

void SwarmAgent::append_closepointpair(int j, ClosePoint *closest)
{
    obs_closet_point_num++;
    closepoint_map.insert(j,closest);
}
