#ifndef CONTROLLERBASE_H
#define CONTROLLERBASE_H

template<typename Model>
struct ControllerBase
{
    ControllerBase() : model{ nullptr } {}

    void set_model(Model * new_model) { model = new_model; }
    Model * get_model() { return model; }

    bool model_is_set() { return model != nullptr; }

private:
    Model * model;
};

#endif // CONTROLLERBASE_H
