#ifndef MAIN_STREAM_H
#define MAIN_STREAM_H

#include <QObject>
#include <mypainter.h>
#include <simulator.h>
class main_stream : public QObject
{
    Q_OBJECT
public:
    explicit main_stream(QObject *parent = nullptr);
    MYpainter *paint;
    Simulator *m_sim;
    void setupEvent();
signals:

};

#endif // MAIN_STREAM_H
