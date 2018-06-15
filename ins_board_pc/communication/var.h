#ifndef VAR_H
#define VAR_H

#include "communication/terminalbase.h"

#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <QObject>
#include <QEventLoop>

class Var
{
public:
    Var(TerminalBase & tbase);

    template<typename T>
    void set(const std::string & name, const T & val)
    {
        tbase.send_text(name + boost::lexical_cast<std::string>(val) + '\n');
    }

    template<typename T>
    T get(const std::string & name)
    {
        std::vector<std::string> tokens;
        tbase.send_text(name + '\n');
        while(!tbase.text_pending()) {}
        boost::split(tokens, tbase.read_text(), boost::is_any_of(" \n"));
        return boost::lexical_cast<T>(tokens[1]);
    }

private:
    TerminalBase & tbase;
};

#endif // VAR_H
