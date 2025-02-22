package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerConstants {
    // Motion Magic Constraints

    private static final double MAX_VELOCITY = 16; // Rotations per second
    private static final double MAX_ACCELERATION = MAX_VELOCITY * 4;
    private static final double MAX_JERK = MAX_ACCELERATION * 4;

    // States

    public enum IndexerState {
        DISABLED(0),
        COLLECTING(MAX_VELOCITY),
        PASSING(MAX_VELOCITY),
        EJECTING(-MAX_VELOCITY);

        public final double velocity;

        IndexerState(double velocity) {
            this.velocity = velocity;
        }
    }

    // Configuration

    public static final TalonFXConfiguration configuration = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(MAX_VELOCITY)
                .withMotionMagicAcceleration(MAX_ACCELERATION)
                .withMotionMagicJerk(MAX_JERK)
        ).withSlot0(
            new Slot0Configs()
                .withKP(0.083374)
                .withKS(0.17818)
                .withKV(0.12464)
                .withKA(0.0039997)
        ).withAudio(
            new AudioConfigs()
                .withBeepOnBoot(true)
                .withBeepOnConfig(true)
        );

}
