#ifndef REMOTEBASE_H
#define REMOTEBASE_H

#include <QObject>

class FilteredPacket;

/*!
 * @brief The RemoteBase class
 * This is the non-template part of any remote filtering controller.
 */
class RemoteBase : public QObject
{
    Q_OBJECT

public:
    /*!
      * @brief RemoteBase constructor.
      */
    RemoteBase() = default;

    /*!
      * @brief RemoteBase destructor.
      */
    virtual ~RemoteBase() = default;

public slots:
    /*!
     * @brief Handle input filtered packet.
     * @param z input filtered packet.
     */
    virtual void handle_input(const FilteredPacket & z) = 0;

    /*!
     * @brief Set adapter's accumulator capacity.
     * @param cap new capacity.
     */
    virtual void set_accum_capacity(const QString & cap) = 0;

    /*!
     * @brief Clear plots.
     */
    virtual void clear_plots() = 0;

private:
    bool filtering_enabled;
};

#endif // REMOTEBASE_H
