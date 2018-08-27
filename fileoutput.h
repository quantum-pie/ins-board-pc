#ifndef FILEOUTPUT_H
#define FILEOUTPUT_H

#include "controllers/runningflag.h"

#include <QObject>

#include <fstream>

class RawPacket;

/*!
 * @brief The FileOutput class
 */
class FileOutput : public QObject, RunningFlag
{
    Q_OBJECT

public:
    /*!
     * @brief FileOutput constructor.
     * @param enable_button Enable widget (optional).
     */
    FileOutput();

    using RunningFlag::is_running;
    using RunningFlag::set_running;

public slots:
    /*!
     * @brief Handle raw input.
     * @param z raw input packet.
     */
    void handle_input(const RawPacket & z);

    /*!
     * @brief Handle enable.
     * @param en enable boolean condition.
     */
    void handle_enable(bool en);

private:
    std::ofstream output_file;

    static constexpr std::size_t batch_size { 10 * 1024 * 1024 };
};

#endif // FILEOUTPUT_H
