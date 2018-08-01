#ifndef RPYTEXTVIEW_H
#define RPYTEXTVIEW_H

#include "views/base/IBaseView.h"
#include "core/IOrientationProvider.h"

class QLineEdit;

/*!
 * @brief The numerical orientation view.
 */
struct StdOrientationView : IOrientationView
{
    /*!
     * @brief StdOrientationView constructor.
     * @param roll_std_le roll angle std LineEdit.
     * @param pitch_std_le pitch angle std LineEdit.
     * @param yaw_std_le yaw angle std LineEdit.
     * @param roll_le roll angle LineEdit.
     * @param pitch_le pitch angle LineEdit.
     * @param magnetic_heading_le magnetic heading LineEdit.
     */
    StdOrientationView(QLineEdit * roll_std_le, QLineEdit * pitch_std_le, QLineEdit * yaw_std_le, QLineEdit * roll_le, QLineEdit * pitch_le, QLineEdit * magnetic_heading_le);

    ~StdOrientationView() override = default;

    void update(const ViewModel & pvd) override;
    void clear() override;

private:
    QLineEdit * roll_std_le;
    QLineEdit * pitch_std_le;
    QLineEdit * yaw_std_le;
    QLineEdit * roll_le;
    QLineEdit * pitch_le;
    QLineEdit * magnetic_heading_le;
};

#endif // RPYTEXTVIEW_H
