#ifndef RUNNINGFLAG_H
#define RUNNINGFLAG_H

struct RunningFlag
{
    explicit RunningFlag(bool flg = true) : running{ flg } {}

    void set_running(bool en) { running = en; }
    bool is_running() { return running; }

private:
    bool running;
};

#endif // RUNNINGFLAG_H
