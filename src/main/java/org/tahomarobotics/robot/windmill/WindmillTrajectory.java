/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.windmill;

import edu.wpi.first.math.Pair;
import org.tahomarobotics.robot.util.motion.MotionProfile;
import org.tahomarobotics.robot.util.motion.MotionState;
import org.tahomarobotics.robot.util.motion.TrapezoidalMotionProfile;
import org.tinylog.Logger;

import java.util.Optional;

public class WindmillTrajectory {

    protected record WindmillProfile(MotionProfile elev, MotionProfile arm) {}

    public final String name;
    private final WindmillProfile[] profiles;
    public final WindmillState[] states;

    MotionState e = new MotionState();
    MotionState a = new MotionState();

    public WindmillTrajectory(String name, WindmillState[] states) throws MotionProfile.MotionProfileException {
        this.name = name;
        this.states = states;
        if (states == null || states.length < 2) {
            throw new IllegalArgumentException("states either null or less than two");
        }


        profiles = new WindmillProfile[states.length - 1];
        WindmillProfile prior = null;
        for (int i = 0; i < profiles.length; i++) {
            prior = profiles[i] = createProfile(prior, i, states, WindmillConstraints.CORAL_CONSTRAINTS); // Default to coral constraints

        }
    }

    public WindmillTrajectory(String name, WindmillState[] states, WindmillConstraints constraints) throws MotionProfile.MotionProfileException {
        this.name = name;
        this.states = states;
        if (states == null || states.length < 2) {
            throw new IllegalArgumentException("states either null or less than two");
        }


        profiles = new WindmillProfile[states.length - 1];
        WindmillProfile prior = null;
        for (int i = 0; i < profiles.length; i++) {
            prior = profiles[i] = createProfile(prior, i, states, constraints);

        }
    }


    private WindmillProfile createProfile(WindmillProfile prior, int index, WindmillState[] states, WindmillConstraints constraints) throws MotionProfile.MotionProfileException {
        double startTime = 0;
        double elevStartVelocity = 0;
        double armStartVelocity = 0;
        if (prior != null) {
            startTime = prior.elev.getEndTime();
            elevStartVelocity = prior.elev.getLastMotionState().velocity;
            armStartVelocity = prior.arm.getLastMotionState().velocity;
        }


        // determine ending velocities
        double elevEndVelocity = 0.0;
        double armEndVelocity = 0.0;

        if (states.length > index + 2) {

            double currentDirection = Math.signum(states[index + 1].elevatorState().heightMeters() - states[index].elevatorState().heightMeters());
            double nextDirection = Math.signum(states[index + 2].elevatorState().heightMeters() - states[index + 1].elevatorState().heightMeters());
            if (currentDirection == nextDirection) {
                elevEndVelocity = constraints.elevMaxVel;
            }

            currentDirection = Math.signum(states[index + 1].armState().angleRadians() - states[index].armState().angleRadians());
            nextDirection = Math.signum(states[index + 2].armState().angleRadians() - states[index + 1].armState().angleRadians());
            if (currentDirection == nextDirection) {
                armEndVelocity = constraints.armMaxVel;
            }
        }


        MotionProfile elev = new TrapezoidalMotionProfile(
            startTime,
            states[index].elevatorState().heightMeters(),
            states[index + 1].elevatorState().heightMeters(),
            elevStartVelocity, elevEndVelocity,
            constraints.elevMaxVel, constraints.elevMaxAccel
        );

        MotionProfile arm = new TrapezoidalMotionProfile(
            startTime,
            states[index].armState().angleRadians(),
            states[index + 1].armState().angleRadians(),
            armStartVelocity, armEndVelocity,
            constraints.armMaxVel, constraints.armMaxAccel
        );

        if (elev.getEndTime() > arm.getEndTime()) {
            arm = arm.updateEndTime(elev.getEndTime());
        } else {
            elev = elev.updateEndTime(arm.getEndTime());
        }

        return new WindmillProfile(elev, arm);
    }

    public WindmillState sample(double time) {
        time = Math.min(Math.max(time, profiles[0].elev.startTime), profiles[profiles.length - 1].elev.getEndTime());

        for (WindmillProfile profile : profiles) {
            if (time <= profile.elev.getEndTime()) {
                profile.elev.getSetpoint(time, e);
                profile.arm.getSetpoint(time, a);
                break;
            }
        }
        return new WindmillState(
            time, new WindmillState.ElevatorState(e.position, e.velocity, e.acceleration),
            new WindmillState.ArmState(a.position, a.velocity, a.acceleration)
        );
    }

    public WindmillState getInitialState() {
        return sample(profiles[0].elev.startTime);
    }

    public double getTotalTimeSeconds() {
        return profiles[profiles.length - 1].elev.getEndTime() - profiles[0].elev.startTime;
    }

    public static Optional<WindmillTrajectory> loadTrajectories(Pair<WindmillConstants.TrajectoryState, WindmillConstants.TrajectoryState> fromTo) {
        Logger.info("Loading trajectory {}", fromTo);


        return Optional.empty();
    }

    public record WindmillConstraints(double armMaxVel, double elevMaxVel, double armMaxAccel, double elevMaxAccel) {
        public static final WindmillConstraints CORAL_CONSTRAINTS = new WindmillConstraints(
            WindmillConstants.ARM_MAX_VELOCITY,
            WindmillConstants.ELEVATOR_MAX_VELOCITY,
            WindmillConstants.ARM_MAX_ACCELERATION,
            WindmillConstants.ELEVATOR_MAX_ACCELERATION);
        public static final WindmillConstraints ALGAE_CONSTRAINTS = new WindmillConstraints(
            WindmillConstants.ARM_MAX_VELOCITY,
            WindmillConstants.ELEVATOR_MAX_VELOCITY,
            WindmillConstants.ARM_MAX_ACCELERATION - WindmillConstants.ARM_ALGAE_ACCELERATION_REDUCTION,
            WindmillConstants.ELEVATOR_MAX_ACCELERATION);
    }
}
