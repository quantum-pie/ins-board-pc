#ifndef SIMPOSITIONATTRCONTROLLER_H
#define SIMPOSITIONATTRCONTROLLER_H

#include "core/ISimPositionAttr.h"
#include "controllers/direct/attributes/attrcontrollerbase.h"

#include <QObject>

class QLineEdit;

/*!
 * @brief The SimPositionAttrController class
 * This is the controller of position filtering simulator attributes.
 */
class SimPositionAttrController : public QObject, AttrControllerBase<ISimPositionAttr>
{
    Q_OBJECT

public:
    /*!
     * @brief SimPositionAttrController constructor.
     * @param speed_le Movement speed LineEdit.
     * @param initial_track_le Initial track angle LineEdit.
     */
    SimPositionAttrController(QLineEdit * speed_le, QLineEdit * initial_track_le);

    //! Bring model setter to scope.
    using AttrControllerBase<ISimPositionAttr>::set_model;

    /*!
     * @brief Apply attributes to the current model.
     */
    void apply_attributes();

    /*!
     * @brief Borrow attributes from the current model.
     */
    void borrow_attributes();

public slots:
    /*!
     * @brief Movement speed changed.
     * @param str new speed string.
     */
    void on_speed(const QString & str);

    /*!
     * @brief Track angle changed.
     * @param str new track angle string.
     */
    void on_initial_track(const QString & str);

private:
    QLineEdit * speed_le;
    QLineEdit * initial_track_le;
};

#endif // SIMPOSITIONATTRCONTROLLER_H
