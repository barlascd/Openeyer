#include "AITrainer.hpp"
#include <spdlog/spdlog.h>
#include <numeric>
#include <random>
#include <algorithm>
#include <stdexcept>

// ─────────────────────────────────────────────────────────────────────────────
// PolicyNet
// ─────────────────────────────────────────────────────────────────────────────

PolicyNet::PolicyNet(int in, int h, int out) {
    fc1 = register_module("fc1", torch::nn::Linear(in, h));
    fc2 = register_module("fc2", torch::nn::Linear(h, h));
    fc3 = register_module("fc3", torch::nn::Linear(h, out));
}

torch::Tensor PolicyNet::forward(torch::Tensor x) {
    x = torch::relu(fc1->forward(x));
    x = torch::relu(fc2->forward(x));
    return torch::tanh(fc3->forward(x)); // FIX: was 'gc3' (typo) → 'fc3'
}

// ─────────────────────────────────────────────────────────────────────────────
// AITrainer
// ─────────────────────────────────────────────────────────────────────────────

AITrainer::AITrainer(int state_dim, int action_dim, int hidden, size_t replay_size)
    : state_dim_(state_dim),
      action_dim_(action_dim),
      replay_size_(replay_size), // FIX: was 'replay_size' (missing underscore)
      policy_(std::make_shared<PolicyNet>(state_dim, hidden, action_dim)),
      optimizer_(policy_->parameters(), torch::optim::AdamOptions(1e-3))
{
    spdlog::info("AITrainer init: state={} action={}", state_dim, action_dim);
}

void AITrainer::addExperience(const Experience& exp) {
    if (replay_buffer_.size() >= replay_size_) // FIX: replay_size → replay_size_
        replay_buffer_.pop_front();
    replay_buffer_.push_back(exp);
}

float AITrainer::trainStep(int batch_size) {
    if (static_cast<int>(replay_buffer_.size()) < batch_size) return 0.0f;

    // Random mini-batch indices
    std::vector<int> idx(replay_buffer_.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::shuffle(idx.begin(), idx.end(), std::mt19937{std::random_device{}()});
    idx.resize(static_cast<size_t>(batch_size));

    // Build tensors
    std::vector<torch::Tensor> states, actions, rewards;
    states.reserve(batch_size);
    actions.reserve(batch_size);
    rewards.reserve(batch_size);

    for (int i : idx) {
        const auto& e = replay_buffer_[i];
        states.push_back(torch::tensor(e.state));
        actions.push_back(torch::tensor(e.action));
        rewards.push_back(torch::tensor(e.reward));
    }

    auto state_batch  = torch::stack(states);
    auto action_batch = torch::stack(actions);
    auto reward_batch = torch::stack(rewards).unsqueeze(1);

    // Forward pass
    auto pred = policy_->forward(state_batch);

    // Behaviour-cloning loss (imitate PID output)
    auto loss = torch::mse_loss(pred, action_batch);

    // Reward-weighted loss: high-reward samples contribute more
    auto weighted_loss = (loss * reward_batch.abs()).mean();

    // Backward pass
    optimizer_.zero_grad();
    weighted_loss.backward();
    torch::nn::utils::clip_grad_norm_(policy_->parameters(), 1.0);
    optimizer_.step();

    return weighted_loss.item<float>();
}

void AITrainer::save(const std::string& path) {
    torch::save(policy_, path);
    spdlog::info("Model saved: {}", path);
}

void AITrainer::load(const std::string& path) {
    torch::load(policy_, path);
    spdlog::info("Model loaded: {}", path);
}

std::vector<float> AITrainer::predict(const std::vector<float>& state) {
    torch::NoGradGuard no_grad;
    auto input  = torch::tensor(state).unsqueeze(0);
    auto output = policy_->forward(input).squeeze(0);
    return std::vector<float>(
        output.data_ptr<float>(),
        output.data_ptr<float>() + output.numel());
}
