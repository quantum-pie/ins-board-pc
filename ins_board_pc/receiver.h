#ifndef RECEIVER_H
#define RECEIVER_H

#include <QObject>

class FilterInput;

class Receiver : public QObject
{
signals:
    void raw_sample_received(const FilterInput & z);
}

#endif // RECEIVER_H
