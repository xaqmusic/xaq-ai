#pragma once
// ConstantSource -- a SCAFFOLD, named as one: publishes a fixed vector on a topic every tick, optionally only inside a
// tick window (another vector outside it).  It exists to drive a handle in a bench without a brain behind it -- the
// stance depth handle's admission (2026-11-08): "crouch the front pair 2 cm from tick 300 to 900, and show the FSR
// shares and the support model's margin move".  Nothing learned, nothing sensed.  Absent = byte-identical.
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class ConstantSource : public Module {
public:
    ConstantSource();
    ~ConstantSource() override;
    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override { return {}; }
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;
    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json diag_snapshot() const override;

private:
    std::string topic_ = "constant.out", sensor_ = "constant";
    std::vector<double> values_, off_values_;
    int64_t from_tick_ = 0, to_tick_ = 0;   // to_tick 0 = forever
    int64_t published_ = 0, in_window_ = 0;
    Bus* bus_ = nullptr;
};

} // namespace ogma
