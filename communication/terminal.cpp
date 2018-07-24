#include "communication/terminal.h"
#include "communication/terminalbase.h"

#include <QScrollBar>

#include <algorithm>

Terminal::Terminal(TerminalBase & tbase)
    : tbase{ tbase }
{
    connect(&tbase, SIGNAL(text_received(std::string)), this, SLOT(print_text(std::string)));

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
    tbase.send_text(std::string(1, ch));
}

void Terminal::showEvent(QShowEvent *event)
{
    clear();
    QPlainTextEdit::showEvent(event);
}

void Terminal::print_text(const std::string & data)
{
    QString flt;
    std::copy_if(data.begin(), data.end(), std::back_inserter(flt), [](char ch){ return ch != '\b'; });
    textCursor().insertText(flt);
    scroll_down();
}

void Terminal::scroll_down() const
{
    QScrollBar *vbar = verticalScrollBar();
    vbar->setValue(vbar->maximum());
}
