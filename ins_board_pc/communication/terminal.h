#ifndef TERMINAL_H
#define TERMINAL_H

#include <QPlainTextEdit>

#include <regex>

class TerminalBase;

class Terminal : public QPlainTextEdit
{
    Q_OBJECT

public:
    Terminal(TerminalBase & tbase);

private slots:
    /* print recieved info in console */
    void print_text(const std::string & str);

private:
    /* redefinition of events */
    void keyPressEvent(QKeyEvent *);
    void mousePressEvent(QMouseEvent *);
    void mouseDoubleClickEvent(QMouseEvent *);
    void contextMenuEvent(QContextMenuEvent *);
    void showEvent(QShowEvent *);

    /* scroll down to the cursor */
    void scroll_down() const;

    TerminalBase & tbase;
};

#endif // TERMINAL_H
