#ifndef RPYREMOTEVIEW_H
#define RPYREMOTEVIEW_H

#include "views/base/IBaseView.h"
#include "packets.h"

class QCustomPlot;

struct RPYRemoteView : IRemoteView
{
    RPYRemoteView(QCustomPlot * plot);
    ~RPYRemoteView() override = default;

    void update(const ViewModel & pvd) override;
    void clear() override;

private:
    QCustomPlot * plot;
};

#endif // RPYREMOTEVIEW_H
