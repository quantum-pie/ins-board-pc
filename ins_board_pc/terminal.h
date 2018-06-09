#ifndef TERMINAL_H
#define TERMINAL_H

#include "udpsender.h"
#include "udpreceiver.h"

#include <QPlainTextEdit>

#include <regex>

class Terminal : public QPlainTextEdit
{
    Q_OBJECT

public:
    Terminal(const QString & client_ip, const QString & server_ip,
             uint16_t client_port, uint16_t server_port);

    /* print recieved info in console */
    void print(const QByteArray & ar);

private:
    /* redefinition of events */
    void keyPressEvent(QKeyEvent *);
    void mousePressEvent(QMouseEvent *);
    void mouseDoubleClickEvent(QMouseEvent *);
    void contextMenuEvent(QContextMenuEvent *);
    void showEvent(QShowEvent *);

    /* scroll down to the cursor */
    void scroll_down() const;

    UDPSender output_sock;
    UDPReceiver input_sock;

    static const std::regex alpha_filter;
};

#endif // TERMINAL_H
