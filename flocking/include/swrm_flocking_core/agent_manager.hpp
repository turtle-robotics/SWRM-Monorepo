#ifndef SWRM_FLOCKING_AGENT_MANAGER_HPP
#define SWRM_FLOCKING_AGENT_MANAGER_HPP

#include <ranges>
#include <istream>
#include <thread>
#include <latch>
#include <chrono>
#include <string>
#include <sstream>
#include <algorithm>
#include <functional>
#include <optional>
#include <stop_token>
#include <tuple>
#include <utility>
#include <vector>

#include "swrm_flocking_core/alpha_agent.hpp"
#include "swrm_flocking_core/gamma_agent.hpp"
#include "swrm_flocking_core/directed_pointer.hpp"

namespace swrm::flocking {
    template <template <typename, typename...> class AgentContainer>
    requires std::ranges::random_access_range<AgentContainer<alpha_agent>>
    && requires(size_t n) {
        AgentContainer<alpha_agent>(n);
    }
    // TODO: Refactor to be multithreaded using directed_pointer and a thread pool
    class agent_manager {
    public:
        using agent_container = AgentContainer<alpha_agent>;
        using agent_iterator = decltype(std::ranges::cbegin(std::declval<agent_container&>()));

    private:
        agent_container alphas;
        std::optional<directed_reader<gamma_agent>> gamma;
        std::jthread gamma_thread;
        std::latch finish_setup;
        std::chrono::duration<double> time_step;
        
        template <typename... GammaArgs>
        void initialize_gamma(GammaArgs&&... gamma_args) {
            this->gamma_thread = std::jthread(
                [this, gamma_args...](std::stop_token stop_token) {
                    directed_pointer<gamma_agent> gamma_ptr = make_directed<gamma_agent>(std::move(gamma_args)...);
                    double sleep_time;
                    {
                        directed_guard gamma_guard = gamma_ptr.lock();
                        sleep_time = gamma_guard->time_delta();
                    }

                    this->gamma.emplace(gamma_ptr.make_reader());
                    this->finish_setup.count_down();

                    while (!stop_token.stop_requested()) {
                        {
                            directed_guard gamma_guard = gamma_ptr.lock();
                            gamma_guard->update();
                        }
                        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
                    }
                });
        }

        std::tuple<int, vector2d, vector2d> read_state(std::istream& in) {
            int id = -1;
            hpscalar px, py, vx, vy;

            std::string line;
            std::getline(in, line);
            if (line == "STOP") {
                return std::make_tuple(-1, vector2d(0.0, 0.0), vector2d(0.0, 0.0));
            }
            std::istringstream iss(line);
            iss >> id >> px >> py >> vx >> vy;

            vector2d position(px, py);
            vector2d velocity(vx, vy);

            return std::make_tuple(id, position, velocity);
        }

        void halt() {
            // TODO: Make this dampen the velocity
            // At the moment this just makes the agents have constant velocity
            for (alpha_agent& agent : this->alphas) {
                agent.update(vector2d(0.0, 0.0));
            }
        }

    public:
        template <typename... GammaArgs>
        agent_manager(
                std::istream& in, 
                size_t alpha_count, 
                double time_step = DEFAULT_TIME_STEP,
                GammaArgs&&... gamma_args
                ) : 
                alphas(alpha_count), 
                time_step(std::chrono::duration<double>(time_step)),
                finish_setup(1) {
            this->initialize_gamma(std::forward<GammaArgs>(gamma_args)...);
        }

        void read_data(std::istream& in) {
            // Read initial robot states from the input stream
            std::vector<bool> initialized(this->alphas.size(), false);
            while (!std::ranges::all_of(initialized, std::identity{})) {
                auto [id, position, velocity] = this->read_state(in);
                if (id == -1) return; // Stop reading if "STOP" is encountered
                if (id < 0 || static_cast<size_t>(id) >= this->alphas.size()) continue; // Ignore invalid IDs

                this->alphas[id].update(position, velocity);
                initialized[id] = true;
            }
            // Wait for gamma agent to set up
            this->finish_setup.wait();
            
            // Spin infinitely
            while (true) {
                auto start_time = std::chrono::steady_clock::now();
                // For the current time stamp
                while (std::chrono::steady_clock::now() - start_time < this->time_step) {
                    // Spin until goodbit
                    if (!in.good()) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                    auto [id, position, velocity] = this->read_state(in);
                    if (id == -1) {
                        this->halt();
                        return; // Stop reading if "STOP" is encountered
                    }
                    if (id < 0 || static_cast<size_t>(id) >= this->alphas.size()) continue; // Ignore invalid IDs

                    this->alphas[id].update(position, velocity);
                }

                // Update all alpha agents based on current agent states
                for (alpha_agent& agent : this->alphas) {
                    vector2d gamma_control(0.0, 0.0);
                    {
                        directed_guard gamma_guard = this->gamma->lock();
                        gamma_control = agent.gamma_agent_control(*gamma_guard);
                    }
                    vector2d inter_agent_control = agent.inter_agent_control(this->alphas);
                    vector2d total_control = gamma_control + inter_agent_control;
                    agent.update(total_control);
                }
            }
        }

        agent_iterator begin() const {
            return std::ranges::cbegin(this->alphas);
        }

        agent_iterator end() const {
            return std::ranges::cend(this->alphas);
        }

        ~agent_manager() {
            this->halt();
            this->gamma_thread.request_stop();
        }
    };
}

#endif // SWRM_FLOCKING_AGENT_MANAGER_HPP
