#include "Policy.hpp"

#include <stdexcept>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace mjhost {

struct Policy::Impl {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "mjhost"};
    Ort::SessionOptions options;
    std::unique_ptr<Ort::Session> session;
    Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::string input_name, output_name;
};

namespace {

// The trailing dimension is the one that encodes the contract; the leading one is
// the batch and is usually dynamic (-1), so only the last is checked.
void check_width(const std::vector<int64_t>& shape, int64_t expected, const char* what,
                 const std::string& path) {
    const int64_t got = shape.empty() ? -1 : shape.back();
    if (got != expected) {
        throw std::runtime_error(path + ": " + what + " is " + std::to_string(got) +
                                 ", expected " + std::to_string(expected));
    }
}

}  // namespace

Policy::Policy(const std::string& onnx_path) : impl_(std::make_unique<Impl>()), path_(onnx_path) {
    // One inference thread. The networks are small enough that a pool costs more in
    // synchronisation than it recovers, and the control thread should not be
    // blocking on a pool it does not own.
    impl_->options.SetIntraOpNumThreads(1);
    impl_->options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    impl_->session = std::make_unique<Ort::Session>(impl_->env, onnx_path.c_str(), impl_->options);

    if (impl_->session->GetInputCount() != 1 || impl_->session->GetOutputCount() != 1) {
        throw std::runtime_error(onnx_path + ": expected one input and one output");
    }
    Ort::AllocatorWithDefaultOptions alloc;
    impl_->input_name  = impl_->session->GetInputNameAllocated(0, alloc).get();
    impl_->output_name = impl_->session->GetOutputNameAllocated(0, alloc).get();

    check_width(impl_->session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape(),
                kObsLen, "observation width", onnx_path);
    check_width(impl_->session->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape(),
                kActionLen, "action count", onnx_path);

    // Warm up. The first inference is always an outlier — lazy initialisation, cold
    // pages, first-touch faults — and paying that on tick one looks exactly like a
    // control loop that missed its deadline.
    infer(std::array<float, kObsLen>{});
}

Policy::~Policy() = default;

std::array<float, kActionLen> Policy::infer(const std::array<float, kObsLen>& obs) {
    const std::array<int64_t, 2> shape{1, kObsLen};
    Ort::Value input = Ort::Value::CreateTensor<float>(
        impl_->memory, const_cast<float*>(obs.data()), obs.size(), shape.data(), shape.size());

    const char* in[]  = {impl_->input_name.c_str()};
    const char* out[] = {impl_->output_name.c_str()};
    auto results = impl_->session->Run(Ort::RunOptions{nullptr}, in, &input, 1, out, 1);

    const auto count = results[0].GetTensorTypeAndShapeInfo().GetElementCount();
    if (count != kActionLen) {
        throw std::runtime_error(path_ + ": " + std::to_string(count) + " actions, expected " +
                                 std::to_string(kActionLen));
    }
    const float* data = results[0].GetTensorData<float>();
    std::array<float, kActionLen> action{};
    for (int i = 0; i < kActionLen; ++i) action[i] = data[i];
    return action;
}

}  // namespace mjhost
