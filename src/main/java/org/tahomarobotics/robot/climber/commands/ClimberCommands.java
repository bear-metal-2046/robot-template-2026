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

package org.tahomarobotics.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.climber.Climber;
import org.tahomarobotics.robot.lights.LED;
import org.tahomarobotics.robot.windmill.Windmill;
import org.tinylog.Logger;

import java.util.Optional;

public class ClimberCommands {
    private static final Climber climber = Climber.getInstance();

    public static Command getClimberCommand() {
        return Commands.deferredProxy(
            () ->
                switch (climber.getClimbState()) {
                    case STOWED -> //Commands.runOnce(climber::disengageSolenoid)
//                                           .andThen(Commands.runOnce(climber::wiggle))
//                                           .andThen(Commands.waitUntil(climber::isAtTargetPosition))
                                           Commands.runOnce(climber::deploy)
                                               .andThen(() -> Optional.ofNullable(CommandScheduler.getInstance().requiring(Windmill.getInstance())).ifPresent(Command::cancel))
                                               .andThen(Commands.runOnce(() -> Windmill.getInstance().setArmPosition(2 * Math.PI / 3)))
                                           .andThen(Commands.waitUntil(Windmill.getInstance()::isArmAtPosition))
                                           .andThen(Commands.runOnce(() -> Windmill.getInstance().setElevatorHeight(0.005)));
//                                           .andThen(Commands.waitUntil(climber::isAtTargetPosition))
//                                           .andThen(Commands.runOnce(climber::engageSolenoid));
                    case DEPLOYED -> Commands.runOnce(climber::climb)
                                             .andThen(Commands.runOnce(() -> LED.getInstance().climb()))
                                             .andThen(Commands.runOnce(() -> Windmill.getInstance().setArmPosition(Math.PI)))
                                             .andThen(Commands.waitUntil(Windmill.getInstance()::isArmAtPosition))
                                             .andThen(Commands.waitUntil(climber::isAtTargetPosition))
                                             .andThen(Commands.runOnce(climber::disableClimberMotors))
                                             .andThen(Commands.runOnce(climber::engageSolenoid));
                    case CLIMBED -> Commands.runOnce(() -> Logger.error("You got one shot to climb and you missed it."));
                    case ZEROED -> Commands.runOnce(climber::disengageSolenoid)
                                           .andThen(Commands.runOnce(climber::wiggle))
                                           .andThen(Commands.waitUntil(climber::isAtTargetPosition))
                                           .andThen(Commands.runOnce(climber::stow))
                                           .andThen(Commands.waitUntil(climber::isAtTargetPosition))
                                           .andThen(Commands.runOnce(climber::engageSolenoid));
                }
        );
    }
}
