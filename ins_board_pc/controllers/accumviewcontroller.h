#ifndef ACCUMVIEWCONTROLLER_H
#define ACCUMVIEWCONTROLLER_H

#include "views/IAccumView.h"

#include <QObject>

#include <vector>
#include <memory>

class QLineEdit;

class AccumViewController : public QObject
{
    Q_OBJECT

public:
    AccumViewController(QLineEdit * samples_le);

    void attach_view(std::shared_ptr<IAccumView> view);
    void remove_views();

public slots:
    void accum_capacity_changed(const QString & str);

private:
   static const std::size_t default_capacity;

   std::vector<std::shared_ptr<IAccumView>> views;
};

#endif // ACCUMVIEWCONTROLLER_H
