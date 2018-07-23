#ifndef BASEMODELSWITCH_H
#define BASEMODELSWITCH_H

#include <QObject>

class QComboBox;

/*!
 * @brief The ModelSwitchBase class
 * This is the base class for all model switches.
 */
class ModelSwitchBase : public QObject
{
    Q_OBJECT

public:
    /*!
     * @brief ModelSwitchBase constructor.
     * @param sw Switch widget.
     */
    ModelSwitchBase(QComboBox * sw);

    /*!
     * @brief ModelSwitchBase destructor.
     */
    virtual ~ModelSwitchBase() = default;

    /*!
     * @brief Enable switch.
     */
    void enable();

    /*!
     * @brief Disable switch.
     */
    void disable();

public slots:
    /*!
     * @brief Switch model.
     * @param idx model index.
     */
    virtual void switch_model(int idx) = 0;

private:
    QComboBox * sw;
};

#endif // BASEMODELSWITCH_H
