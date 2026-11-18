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
    /**
     * Coordinates a fixed set of alpha agents and one gamma agent.
     *
     * The manager owns the alpha agents and starts a background thread that
     * advances the gamma agent. Call read_data() to ingest agent states and run
     * the control loop. The gamma thread stops when the manager is destroyed.
     *
     * `AgentContainer` must be constructible from a size and provide a
     * random-access range of alpha_agent values. Access to the alpha agents is
     * not synchronized with read_data(), so do not iterate the manager while
     * that method is running.
     *
     * @tparam AgentContainer Container template used to store alpha agents.
     */
    template <template <typename, typename...> class AgentContainer>
    requires std::ranges::random_access_range<AgentContainer<alpha_agent>>
    && requires(size_t n) {
        AgentContainer<alpha_agent>(n);
    }
    // TODO: Refactor to be multithreaded using directed_pointer and a thread pool
    class agent_manager {
    public:
        /**
         * Container type that stores the alpha agents.
         */
        using agent_container = AgentContainer<alpha_agent>;
        /**
         * Const iterator type over the managed alpha agents.
         */
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
        /**
         * Creates the alpha agents and starts the gamma agent worker.
         *
         * The alpha agents begin at the origin with zero velocity. The
         * constructor starts the gamma thread but does not read from `in`.
         *
         * @tparam GammaArgs Constructor argument types for gamma_agent.
         * @param in Stream intended for subsequent read_data() calls.
         * @param alpha_count Number of alpha agents to create.
         * @param time_step Control period used by read_data(), in simulation
         * time.
         * @param gamma_args Arguments forwarded to the gamma_agent constructor.
         */
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

        /**
         * Reads agent states and runs the flocking control loop until STOP.
         *
         * During initialization, each state line has the form `id x y vx vy`
         * and must contain all five fields. Reading continues until every alpha
         * agent has reported or `STOP` is read. Invalid IDs are ignored. If
         * `STOP` or the end of the stream is reached before initialization
         * finishes, the method returns without entering the control loop.
         *
         * Once initialized, the method spends each control period ingesting
         * state updates, then computes gamma and inter-agent controls and
         * stores them as alpha-agent acceleration commands. In the control
         * loop, it returns after reading `STOP` and clears those commands
         * before returning.
         *
         * The stream must remain readable while the control loop runs. If it
         * reaches a non-good state, the method waits for more input rather than
         * terminating. It must be allowed to return normally before the manager
         * is destroyed.
         *
         * @param in Stream containing `id x y vx vy` lines or `STOP`.
         */
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

        /**
         * Returns a const iterator to the first alpha agent.
         *
         * The range is valid while the manager exists. Do not iterate the
         * agents while read_data() is modifying their state.
         *
         * @return Const iterator to the first agent.
         */
        agent_iterator begin() const {
            return std::ranges::cbegin(this->alphas);
        }

        /**
         * Returns a const iterator one past the last alpha agent.
         *
         * @return Const iterator one past the last agent.
         */
        agent_iterator end() const {
            return std::ranges::cend(this->alphas);
        }

        /**
         * Stops the gamma worker and waits for it to finish.
         *
         * Alpha acceleration commands are cleared, while positions and
         * velocities are left unchanged. Destruction can wait until the gamma
         * agent's current update interval elapses. It must not overlap with
         * read_data().
         */
        ~agent_manager() {
            this->halt();
            this->gamma_thread.request_stop();
        }
    };
}

#endif // SWRM_FLOCKING_AGENT_MANAGER_HPP
