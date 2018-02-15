#include "eigenaux.h"
#include <QDebug>

namespace eaux
{

void debug_vector(const NumVector & vec, QString name)
{
    QDebug deb = qDebug();
    deb << name + ":" << endl;
    for(int i = 0; i < vec.size(); ++i)
    {
        deb << vec[i];
    }
    deb << endl;
}

void debug_matrix(const NumMatrix & mtx, QString name)
{
    QDebug deb = qDebug();
    deb << name + ":" << endl;
    for(int i = 0; i < mtx.rows(); ++i)
    {
        for(int j = 0; j < mtx.cols(); ++j)
        {
            deb << mtx(i, j);
        }
        deb << endl;
    }
}

}

