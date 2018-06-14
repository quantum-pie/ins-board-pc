#ifndef REMOTEBASE_H
#define REMOTEBASE_H

#include <QObject>

class FilteredPacket;

class RemoteBase : public QObject
{
    Q_OBJECT

public:
    RemoteBase() = default;
    virtual ~RemoteBase() = default;

public slots:
    virtual void handle_input(const FilteredPacket &) = 0;
    virtual void set_accum_capacity(const QString & cap) = 0;
    virtual void clear_plots() = 0;

private:
    bool filtering_enabled;
};

#endif // REMOTEBASE_H
