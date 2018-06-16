#ifndef VAR_H
#define VAR_H

#include "communication/terminalbase.h"

#include <string>
#include <vector>
#include <chrono>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <QObject>

class Var
{
public:
    Var(TerminalBase & tbase) : tbase{ tbase }
    {}

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

        auto t1 = std::chrono::high_resolution_clock::now();
        auto t2 = t1;

        while(!tbase.text_pending() &&
              std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() < timeout_ms)
        {
            QCoreApplication::processEvents();
            t2 = std::chrono::high_resolution_clock::now();
        }

        if(tbase.text_pending())
        {
            std::string input = tbase.read_text();
            boost::split(tokens, input, boost::is_any_of(" \n"));
            return boost::lexical_cast<T>(tokens[1]);
        }
        else
        {
            return T{};
        }
    }

private:
    static const int timeout_ms { 50 };
    TerminalBase & tbase;
};

#endif // VAR_H
