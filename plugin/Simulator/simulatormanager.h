#ifndef SIMULATORMANAGER_H
#define SIMULATORMANAGER_H

#include <QObject>
#include <service/SimDynamicsservice.h>
#include <simer.h>
class SimulatorManager : public QObject,public SimDynamicsservice
{
    Q_OBJECT
    Q_INTERFACES(SimDynamicsservice)
public:
    explicit SimulatorManager(QObject *parent = nullptr);
    void set_delta_t(double delta_t) override;
    void start_sim() override;
    void set_ODE(ODE_functor *functor) override;

    double delt_t;
    Simer *m_sim;

signals:

};

#endif // SIMULATORMANAGER_H
