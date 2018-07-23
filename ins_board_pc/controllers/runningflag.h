#ifndef RUNNINGFLAG_H
#define RUNNINGFLAG_H

/*!
 * @brief The RunningFlag struct
 * Boolean flag encapsulation.
 */
struct RunningFlag
{
    /*!
     * @brief RunningFlag constructor.
     * @param flg initial flag value.
     */
    explicit RunningFlag(bool flg = true) : running{ flg } {}

    /*!
     * @brief Set flag value.
     * @param en new flag value.
     */
    void set_running(bool en) { running = en; }

    /*!
     * @brief Check flag state.
     * @return Flag state.
     */
    bool is_running() { return running; }

private:
    bool running;
};

#endif // RUNNINGFLAG_H
