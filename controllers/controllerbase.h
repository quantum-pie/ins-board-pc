#ifndef CONTROLLERBASE_H
#define CONTROLLERBASE_H

/*!
 * @brief This is the base controller of the model.
 */
template<typename Model>
struct ControllerBase
{
    /*!
     * @brief ControllerBase constructor.
     */
    ControllerBase() : model{ nullptr } {}

    /*!
     * @brief Set model.
     * @param new_model new model.
     */
    void set_model(Model * new_model) { model = new_model; }

    /*!
     * @brief Get model.
     * @return current model pointer.
     */
    Model * get_model() { return model; }

    /*!
     * @brief Check if model is set.
     * @return True if model is set.
     */
    bool model_is_set() { return model != nullptr; }

private:
    Model * model;
};

#endif // CONTROLLERBASE_H
