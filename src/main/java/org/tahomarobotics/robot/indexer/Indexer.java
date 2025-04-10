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

package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.AutoLogOutput;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.CollectorConstants;
import org.tahomarobotics.robot.grabber.Grabber;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import org.tahomarobotics.robot.util.sysid.SysIdTests;
import org.tahomarobotics.robot.windmill.Windmill;
import org.tahomarobotics.robot.windmill.WindmillConstants;

import java.util.List;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.tahomarobotics.robot.indexer.IndexerConstants.*;

public class Indexer extends SubsystemIF {
    private static final Indexer INSTANCE = new Indexer();

    // -- Member Variables --

    // Hardware

    private final TalonFX motor;

    private final DigitalInput beanBake;

    // Status Signals

    private final StatusSignal<Current> current;

    private final LoggedStatusSignal[] statusSignals;

    // Control Requests

    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0);

    // State

    @AutoLogOutput(key = "Indexer/State")
    private IndexerState state = IndexerState.DISABLED;

    // -- Initialization --

    private Indexer() {
        // Create hardware

        motor = new TalonFX(RobotMap.INDEXER_MOTOR);

        beanBake = new DigitalInput(RobotMap.BEAM_BREAK);

        // Configure hardware

        RobustConfigurator.tryConfigureTalonFX("Indexer Motor", motor, configuration);

        // Bind status signals

        current = motor.getSupplyCurrent();

        statusSignals = new LoggedStatusSignal[]{
            new LoggedStatusSignal("Position", motor.getPosition()),
            new LoggedStatusSignal("Velocity", motor.getVelocity()),
            new LoggedStatusSignal("Current", current)
        };

        LoggedStatusSignal.setUpdateFrequencyForAll(statusSignals, RobotConfiguration.MECHANISM_UPDATE_FREQUENCY);
        ParentDevice.optimizeBusUtilizationForAll(motor);
    }

    public static Indexer getInstance() {
        return INSTANCE;
    }

    // -- State Machine --

    private void setTargetState(IndexerState state) {
        this.state = state;

        switch (state.type) {
            case NONE -> motor.stopMotor();
            case POSITION -> motor.setControl(positionControl.withPosition(state.value).withSlot(1));
            case VELOCITY -> motor.setControl(velocityControl.withVelocity(state.value).withSlot(0));
        }
    }

    // Transitions

    public void transitionToDisabled() {
        setTargetState(IndexerState.DISABLED);
    }

    public void transitionToCollecting() {
        if (!isArmAtCollecting()) {
            setTargetState(IndexerState.QUEUE_COLLECTING);
            return;
        }
        setTargetState(IndexerState.COLLECTING);
    }

    public void transitionToQueueing() {
        setTargetState(IndexerState.QUEUEING);
    }

    public void transitionToPassing() {
        setTargetState(IndexerState.PASSING);
    }

    public void transitionToEjecting() {
        setTargetState(IndexerState.EJECTING);
    }

    // -- Getter(s) --

    public IndexerState getState() {
        return state;
    }

    public double getCurrent() {
        return current.getValueAsDouble();
    }

    @AutoLogOutput(key = "Indexer/Beam Brake Tripped?")
    public boolean isBeanBakeTripped() {
        return !beanBake.get();
    }

    @AutoLogOutput(key = "Indexer/Is Arm Collecting?")
    private boolean isArmAtCollecting() {
        return Windmill.getInstance().getTargetTrajectoryState() == WindmillConstants.TrajectoryState.CORAL_COLLECT
            && Windmill.getInstance().isAtTargetTrajectoryState();
    }

    // -- Periodic --

    private void stateMachine() {
        if (!isArmAtCollecting() && isBeanBakeTripped()) {
            transitionToQueueing();
            Collector.getInstance().setTargetCollectorState(CollectorConstants.TargetCollectorState.DISABLED);
        } else if ((state == IndexerState.QUEUEING || isBeanBakeTripped()) && isArmAtCollecting()) {
            if (!Grabber.getInstance().isHoldingCoral()) transitionToPassing();
            Grabber.getInstance().transitionToCoralCollecting();
        }
    }

    @Override
    public void periodic() {
        LoggedStatusSignal.refreshAll(statusSignals);
        LoggedStatusSignal.log("Indexer/", statusSignals);
        stateMachine();
    }

    @Override
    public void onDisabledInit() {
        transitionToDisabled();
    }

    // -- SysId --

    @Override
    public List<SysIdTests.Test> getSysIdTests() {
        return List.of(
            SysIdTests.characterize(
                "Indexer SysId Test",
                this,
                motor,
                Volts.of(1).per(Second),
                Volts.of(3)
            ));
    }

}


