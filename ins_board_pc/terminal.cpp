#include "terminal.h"

#include <QScrollBar>

const std::regex Terminal::alpha_filter { R"([[:alpha:]])" };

Terminal::Terminal(const QString & client_ip, const QString & server_ip,
                   uint16_t client_port, uint16_t server_port)
    : output_sock{ server_ip, server_port },
      input_sock{ client_ip, client_port, [this](const QByteArray &ar){ print(ar); } }
{
    setWindowTitle(tr("Terminal"));
    setWindowModality(Qt::ApplicationModal);
    resize(800, 500);
    setTabStopDistance(150);

    QPalette p = palette();
    p.setColor(QPalette::Base, Qt::black);
    p.setColor(QPalette::Text, Qt::green);
    setPalette(p);
}

void Terminal::mouseDoubleClickEvent(QMouseEvent *)
{}

void Terminal::contextMenuEvent(QContextMenuEvent *)
{}

void Terminal::mousePressEvent(QMouseEvent *)
{
    setFocus();
}

void Terminal::keyPressEvent(QKeyEvent *event)
{
    char ch = event->text().at(0).toLatin1();
    if(ch == '\b' && toPlainText().at(textCursor().position() - 1) != '>')
    {
        textCursor().deletePreviousChar();
    }
    output_sock.write_data(&ch, 1);
}

void Terminal::showEvent(QShowEvent *event)
{
    clear();
    QPlainTextEdit::showEvent(event);
}

void Terminal::print(const QByteArray & data)
{
    QString filtered(data);
    textCursor().insertText(filtered.remove('\b'));
    scroll_down();
}

void Terminal::scroll_down() const
{
    QScrollBar *vbar = verticalScrollBar();
    vbar->setValue(vbar->maximum());
}
