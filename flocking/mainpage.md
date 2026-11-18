# SWRM Flocking Code {#mainpage}

SWRM Flocking is a two-dimensional, local-interaction flocking library. It
implements the three steering rules from Reynolds' boids model and adds a
virtual leader, called the gamma agent, that the flock follows. The
\ref swrm::flocking::agent_manager "agent_manager" owns a fixed set of alpha
agents, runs the gamma agent on a background thread, and turns incoming state
reports into acceleration commands.

## Reynolds' flocking principles

Reynolds described flocking as the combination of three local steering rules.
Each alpha agent applies these rules using only nearby agents, so the flock
behaves as a whole without a central controller.

### Separation

Separation keeps agents from crowding each other. When an alpha agent is closer
than `LATTICE_SCALE` to a neighbor, the position term pushes it away from that
neighbor. The repulsion is strongest at short range and fades as the neighbor
approaches the desired spacing.

### Alignment

Alignment turns the flock toward a common heading. The velocity term in
\ref swrm::flocking::alpha_agent::inter_agent_control "inter_agent_control()"
adds the difference between each nearby neighbor's velocity and the agent's own
velocity, weighted by proximity. The result is scaled by
`AGENT_VELOCITY_GAIN`.

### Cohesion

Cohesion keeps the flock together. When an alpha agent is farther than
`LATTICE_SCALE` from a neighbor, the position term pulls it toward that
neighbor. The pull fades to zero at `INTERACTION_RANGE`, so agents only react to
local neighbors.

### Combined interaction term

The implementation uses the Olfati-Saber lattice formulation, in which the
position term supplies both separation and cohesion. For a neighbor at
sigma-norm distance `z`, the position contribution is:

```
inter_agent_action_function(z) * sigma_norm_gradient(q_j - q_i)
```

The action function is negative below `SIGMA_LATTICE_SCALE`, producing
repulsion, and positive above it, producing attraction. A smooth bump weight
removes the action at `SIGMA_INTERACTION_RANGE`. The velocity-alignment term
uses the same spatial weighting. The final inter-agent command is scaled by
`AGENT_POSITION_GAIN` and `AGENT_VELOCITY_GAIN`.

## Gamma agent and navigation

The \ref swrm::flocking::gamma_agent "gamma_agent" is a virtual leader that
moves along a caller-supplied velocity profile. It is not one of Reynolds'
three rules; it gives the flock a navigation target. Each alpha agent tracks
the gamma agent's position through a bounded sigmoid and its velocity through a
direct velocity error. The manager adds this navigational control to the
inter-agent control.

The gamma agent runs in its own thread and advances at its own time step. The
manager's time step is separate: it is the control period used while collecting
alpha-agent state updates.

## Using agent_manager

\ref swrm::flocking::agent_manager "agent_manager" is a class template over the
container used to store alpha agents. The container must be constructible from
a size and provide a random-access range of `alpha_agent` values. `std::vector`
is the usual choice:

```cpp
#include <istream>
#include <vector>

#include <swrm_flocking_core/agent_manager.hpp>
#include <swrm_flocking_core/parameters.hpp>
#include <swrm_flocking_core/types.hpp>

void run_flock(std::istream& states) {
    auto velocity_profile = [](swrm::flocking::hpscalar) {
        return swrm::flocking::vector2d(1.0, 0.0);
    };

    swrm::flocking::agent_manager<std::vector> manager(
        states,
        3,
        swrm::flocking::DEFAULT_TIME_STEP,
        swrm::flocking::vector2d(0.0, 0.0),
        swrm::flocking::vector2d(0.0, 0.0),
        velocity_profile,
        swrm::flocking::DEFAULT_TIME_STEP);

    manager.read_data(states);
}
```

The constructor creates `alpha_count` alpha agents at the origin with zero
velocity and starts the gamma thread. It does not consume the input stream;
pass the same stream to `read_data()`. The manager's `time_step` argument sets
the control period. Any additional constructor arguments after it are
forwarded to the `gamma_agent` constructor, in the example above the initial
position, initial velocity, velocity profile, and gamma time step.

In normal operation, `read_data()` blocks until it reads `STOP`. During
initialization it reads one state per alpha agent; after that it repeatedly
collects state updates for a control period, then computes the gamma and
inter-agent controls and stores their sum as each alpha agent's acceleration
command.

The alpha agents do not integrate their own motion. The manager computes and
stores their acceleration commands, but it does not apply them or write them to
another stream. A surrounding simulation or robot driver supplies the state
updates and is responsible for using the stored commands. Access through
`begin()` and `end()` is not synchronized with `read_data()`, so do not iterate
the manager while that method is running. `STOP` clears the acceleration
commands before `read_data()` returns. The manager's destructor also clears the
commands and stops the gamma thread; let `read_data()` return before destroying
the manager.

## Input stream format

`read_data()` consumes newline-delimited state records. Each state record has
five whitespace-separated fields:

```
id x y vx vy
```

- `id` is the zero-based alpha-agent slot to update. It must be in
  `[0, alpha_count)`. IDs outside that range are ignored.
- `x` and `y` are the agent's position.
- `vx` and `vy` are the agent's velocity.
- The numeric fields are parsed as `hpscalar` values.

The input `id` is a container index, not the identifier returned by
`alpha_agent::ID()`.

For a three-agent manager, a complete initialization followed by a stop looks
like this:

```
0 0.0 0.0 0.0 0.0
1 0.5 0.0 0.0 0.0
2 0.0 0.5 0.0 0.0
STOP
```

Initialization finishes when every valid agent slot has reported at least
once. Records may arrive in any order, and later records for the same slot
replace its earlier state. If `STOP` or the end of the stream is reached before
every slot has reported, `read_data()` returns without entering the control
loop; uninitialized agents remain at the origin with zero velocity.

After initialization, state records are consumed for the duration of each
control period. The stream must remain readable while the control loop runs.
`STOP`, on a line containing exactly that token, is the normal way to end the
loop. The method then clears the acceleration commands, leaves positions and
velocities unchanged, and returns.

If the stream reaches a non-good state after initialization, the manager waits
for more input rather than returning; it does not treat end-of-stream as a
stop. Use `STOP` to terminate the control loop.
