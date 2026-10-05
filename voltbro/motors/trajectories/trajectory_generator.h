#pragma once

/** Initial motion in caller-selected position units and units/second. */
struct TrajectoryState {
    float position;
    float velocity;
};

/** Open-loop scalar reference. Times are seconds; step requires a successful start.
 * Failed start/retarget leaves the current trajectory intact. Reset retains parameters.
 * Protocol identifiers, measurement acquisition and scheduling belong to the caller.
 */
class TrajectoryGenerator {
public:
    virtual ~TrajectoryGenerator() = default;
    virtual bool start(TrajectoryState initial, float target) = 0;
    virtual bool retarget(TrajectoryState initial, float target) = 0;
    /** Publish a successfully started candidate. Active is null on mode entry,
     * otherwise it must be the same concrete type. Called with execution excluded;
     * delay is the time from the initial-state snapshot to the first scheduled step.
     * Implementations may preserve active reference state, never its old goal/settings.
     */
    virtual void on_publish(const TrajectoryGenerator*, float) {}
    /** First scheduled tick after activation; default retains the prepared initial state. */
    virtual void on_activate(TrajectoryState) {}
    virtual float step(float dt) = 0;
    virtual void reset() = 0;
};
