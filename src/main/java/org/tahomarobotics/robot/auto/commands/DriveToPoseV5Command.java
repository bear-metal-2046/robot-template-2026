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

package org.tahomarobotics.robot.auto.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.vision.Vision;
import org.tinylog.Logger;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static org.tahomarobotics.robot.auto.AutonomousConstants.*;

public class DriveToPoseV5Command extends Command {
    // -- Requirements --

    private final Chassis chassis = Chassis.getInstance();

    // -- Controllers --

    private final ProfiledPIDController x, y, r;

    // -- State --

    private Pose2d startPose;

    private final int isolationTarget;
    private Translation2d targetTranslation;
    private Translation2d lastTargetTranslation;
    private Rotation2d targetRotation;
    private final Supplier<Optional<Translation2d>> targetSupplier;

    // -- Initialization --

    public DriveToPoseV5Command(int isolationTarget, Supplier<Optional<Translation2d>> targetSupplier, Translation2d defaultTarget) {
        this.isolationTarget = isolationTarget;
        this.targetSupplier = targetSupplier;
        this.targetTranslation = defaultTarget;

        x = new ProfiledPIDController(
            TRANSLATION_ALIGNMENT_KP, TRANSLATION_ALIGNMENT_KI, TRANSLATION_ALIGNMENT_KD,
            TRANSLATION_ALIGNMENT_CONSTRAINTS
        );
        x.setTolerance(0);

        y = new ProfiledPIDController(
            TRANSLATION_ALIGNMENT_KP, TRANSLATION_ALIGNMENT_KI, TRANSLATION_ALIGNMENT_KD,
            TRANSLATION_ALIGNMENT_CONSTRAINTS
        );
        y.setTolerance(0);

        r = new ProfiledPIDController(
            ROTATION_ALIGNMENT_KP, ROTATION_ALIGNMENT_KI, ROTATION_ALIGNMENT_KD,
            ROTATION_ALIGNMENT_CONSTRAINTS
        );
        r.setTolerance(ROTATION_ALIGNMENT_TOLERANCE);
        r.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(chassis);
    }

    // -- Command --

    @Override
    public void initialize() {
        Pose2d currentPose = chassis.getPose();
        ChassisSpeeds currentVelocity = chassis.getFieldChassisSpeeds();

        Logger.info("Driving to supplied position going through {}", currentPose);

        x.reset(currentPose.getX(), 0);
        y.reset(currentPose.getY(), 0);
        r.reset(currentPose.getRotation().getRadians(), currentVelocity.omegaRadiansPerSecond);

        chassis.setAutoAligning(true);
        Vision.getInstance().isolate(isolationTarget);

        startPose = currentPose;
    }

    @Override
    public void execute() {
//        // If there is too big a jump in the target, don't update it
//        if (targetTranslation.getDistance(targetSupplier.get().orElse(new Translation2d())) < MAX_JUMP_DISTANCE) {
//            return;
//        }

        // Calculate target position and rotation
        Pose2d currentPose = chassis.getPose();
        targetTranslation = targetSupplier.get().orElse(targetTranslation);
        targetRotation = targetTranslation.minus(currentPose.getTranslation()).getAngle();

        syncGoal();

        // Calculate target speeds
        double distanceToGoalPose = currentPose.getTranslation().getDistance(targetTranslation);

        double speedReduction = MathUtil.clamp(distanceToGoalPose, 0.75, 1.0);

        double vx = x.calculate(currentPose.getX()) * speedReduction;
        double vy = y.calculate(currentPose.getY()) * speedReduction;
        double vr = r.calculate(currentPose.getRotation().getRadians());

        // Drive with calculated speeds
        chassis.drive(new ChassisSpeeds(vx, vy, vr), true);

        // Logging
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/X Distance", getVerticalDistanceToWaypoint());
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/Y Distance", getHorizontalDistanceToWaypoint());
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/Target Waypoint", targetTranslation);

        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/vx", vx);
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/vy", vy);
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/vr", vr);
    }

    @Override
    public boolean isFinished() {
        Translation2d robotToTarget = chassis.getPose().getTranslation().minus(targetTranslation);
        return (Math.abs(robotToTarget.getX()) < X_TOLERANCE
                && Math.abs(robotToTarget.getY()) < Y_TOLERANCE
        );
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(new ChassisSpeeds());
        chassis.setAutoAligning(false);

        Vision.getInstance().globalize();

        Logger.info("Finished drive to pose command" + ((interrupted) ? " because it was interrupted." : "!"));
    }

    // -- Instance Methods --

    public double getDistanceFromStart() {
        return startPose.getTranslation().getDistance(chassis.getPose().getTranslation());
    }

    public double getDistanceToWaypoint() {
        return targetTranslation.getDistance(chassis.getPose().getTranslation());
    }

    public double getAngleToWaypoint() {
        return targetRotation.getRadians() - chassis.getPose().getRotation().getRadians();
    }

    public double getHorizontalDistanceToWaypoint() {
        return getDistanceToWaypoint() * Math.cos(getAngleToWaypoint());
    }

    public double getVerticalDistanceToWaypoint() {
        return getDistanceToWaypoint() * Math.sin(getAngleToWaypoint());
    }

    // -- Command Helpers --

    public Command runWhen(BooleanSupplier condition, Command command) {
        return Commands.waitUntil(condition).andThen(command);
    }

    // -- Helpers --

    private void syncGoal() {
        x.setGoal(targetTranslation.getX());
        y.setGoal(targetTranslation.getY());
        r.setGoal(targetRotation.getRadians());
    }
}