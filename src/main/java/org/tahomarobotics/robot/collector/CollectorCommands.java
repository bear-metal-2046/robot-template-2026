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

package org.tahomarobotics.robot.collector;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CollectorCommands {
    public static Command createZeroCommand(Collector collector) {
        return collector.runOnce(collector::setZeroingVoltage)
                        .andThen(Commands.waitSeconds(0.1))
                        .andThen(Commands.waitUntil(collector::isDeploymentStopped))
                        .withTimeout(CollectorConstants.DEPLOYMENT_ZEROING_TIMEOUT)
                        .andThen(collector::zero)
                        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                        .onlyIf(() -> collector.getTargetDeploymentState() == CollectorConstants.TargetDeploymentState.ZEROED);
    }

    public static Command createDeploymentControlCommand(Collector collector) {
        return collector.runOnce(() -> {
            if (collector.isDeploymentStowed()) {
                collector.deploymentTransitionToCollect();
            } else if (collector.isDeploymentCollecting()) {
                collector.deploymentTransitionToStow();
            }
        });
    }

    public static Command createAutoCollectCommand(Collector collector) {
        return collector.runOnce(() -> {
            collector.deploymentTransitionToCollect();
            collector.collectorTransitionToCollecting();
        });
    }

    /** @return On true and on false commands. */
    public static Pair<Command, Command> createCollectorControlCommands(Collector collector) {
        Command onTrue = collector.runOnce(() -> {
            if (collector.isDeploymentCollecting() && collector.isNotHoldingAlgae()) {
                collector.collectorTransitionToCollecting();
            }
        });
        Command onFalse = collector.runOnce(() -> {
            if (collector.getTargetCollectorState() != CollectorConstants.TargetCollectorState.HOLDING_ALGAE) {
                collector.collectorTransitionToDisabled();
            }
        });

        return Pair.of(onTrue, onFalse);
    }

    /** @return On true and on false commands. */
    public static Pair<Command, Command> createEjectCommands(Collector collector) {
        Command onTrue = collector.runOnce(collector::transitionToEjecting);
        Command onFalse = collector.runOnce(collector::cancelEjecting);

        return Pair.of(onTrue, onFalse);
    }
}
