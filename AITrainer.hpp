#pragma once
#include <torch/torch.h>
#include <vector>
#include <deque>
#include <string>

/**
 * @brief Two-hidden-layer policy network.
 *
 * Architecture: Linear → ReLU → Linear → ReLU → Linear → tanh
 * Output range: [-1, 1]
 */
struct PolicyNet : torch::nn::Module {
    PolicyNet(int input_dim, int hidden, int output_dim);
    torch::Tensor forward(torch::Tensor x);

    torch::nn::Linear fc1{nullptr}, fc2{nullptr}, fc3{nullptr};
};

/**
 * @brief Behaviour-cloning trainer with reward-weighted loss.
 *
 * Learns to imitate the PID controller while weighting samples
 * by their observed reward, so high-quality behaviour is reinforced.
 */
class AITrainer {
public:
    struct Experience {
        std::vector<float> state;       ///< Sensor feature vector
        std::vector<float> action;      ///< PID output (target)
        float              reward;      ///< Scalar reward
        std::vector<float> next_state;  ///< Next sensor state
    };

    AITrainer(int state_dim, int action_dim,
              int hidden = 128, size_t replay_size = 10000);

    void  addExperience(const Experience& exp);

    /// Sample a mini-batch and run one gradient step. Returns loss.
    float trainStep(int batch_size = 64);

    void save(const std::string& path);
    void load(const std::string& path);

    /// Inference – returns predicted action for a given state vector.
    std::vector<float> predict(const std::vector<float>& state);

private:
    std::shared_ptr<PolicyNet> policy_;
    torch::optim::Adam         optimizer_;
    std::deque<Experience>     replay_buffer_;
    size_t replay_size_;  // FIX: trailing underscore for consistency with impl
    int    state_dim_, action_dim_;
};
