#ifndef TERMINAL_H
#define TERMINAL_H

#include <QPlainTextEdit>

#include <regex>

class TerminalBase;

/*!
 * @brief The Terminal class
 * This class is designed to communicate with board via terminal.
 */
class Terminal : public QPlainTextEdit
{
    Q_OBJECT

public:
    /*!
     * @brief Terminal constructor.
     * @param tbase Terminal base reference.
     */
    Terminal(TerminalBase & tbase);

private slots:
    /*!
     * @brief Print received text to terminal.
     * @param str received text.
     */
    void print_text(const std::string & str);

private:
    void keyPressEvent(QKeyEvent *);
    void mousePressEvent(QMouseEvent *);
    void mouseDoubleClickEvent(QMouseEvent *);
    void contextMenuEvent(QContextMenuEvent *);
    void showEvent(QShowEvent *);

    void scroll_down() const;

    TerminalBase & tbase;
};

#endif // TERMINAL_H
