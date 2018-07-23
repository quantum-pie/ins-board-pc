#ifndef TERMINALBASE_H
#define TERMINALBASE_H

#include "communication/udpsender.h"
#include "communication/udpreceiver.h"

#include <QObject>

/*!
 * @brief The TerminalBase class.
 * This class is designed to send and receive text messages to/from the board.
 */
class TerminalBase : public QObject
{
    Q_OBJECT

public:
    /*!
     * @brief TerminalBase constructor.
     * @param client_ip Terminal client IP address (PC).
     * @param client_port Terminal client port number.
     * @param server_ip Terminal server IP address (board).
     * @param server_port Terminal server port number.
     */
    TerminalBase(const QString & client_ip, uint16_t client_port,
                 const QString & server_ip, uint16_t server_port);

public slots:
    /*!
     * @brief Send text to board.
     * @param str text to send.
     */
    void send_text(const std::string & str);

signals:
    /*!
     * @brief Receive text from board.
     * @param ar received text.
     */
    void text_received(const std::string & ar);

private:
    UDPSender output_sock;
    UDPReceiver input_sock;
};

#endif // TERMINALBASE_H
