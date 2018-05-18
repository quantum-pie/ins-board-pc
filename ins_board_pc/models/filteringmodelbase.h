#ifndef FILTERINGMODELBASE_H
#define FILTERINGMODELBASE_H

#include <QObject>

class FilteringModelBase : public QObject
{
    Q_OBJECT

public:
    FilteringModelBase() : QObject{ nullptr } {}

signals:
    void refreshed() const;
};

#endif // FILTERINGMODELBASE_H
