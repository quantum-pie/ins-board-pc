#include "communication/terminalbase.h"

TerminalBase::TerminalBase(const QString &client_ip, uint16_t client_port,
                           const QString &server_ip, uint16_t server_port)
    : output_sock{ server_ip, server_port },
      input_sock{ client_ip, client_port, [this](const QByteArray &ar)
                                          { emit text_received(ar.toStdString()); } }
{}

void TerminalBase::send_text(const std::string &str)
{
    output_sock.write_data(str.c_str(), str.length());
}
