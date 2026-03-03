// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class IntakeConstants {
        // Motor ID
        public static final int PIVOT_ID = 1;
        public static final int ROLLER_MOTOR_ID = 2;

        // settings
        public static final boolean kPivotInverted = false;
        public static final boolean kRollerInverted = false;
        public static final IdleMode kPivotIdleMode = IdleMode.kBrake; 
        public static final IdleMode kRollerIdleMode = IdleMode.kCoast;

        // current limits (amps)
        public static final int kPivotCurrentLimit = 40; 
        public static final int kRollerCurrentLimit = 20;

        // PID
        public static final double kP_PIVOT = 0.0; 
        public static final double kI_PIVOT = 0.0;
        public static final double kD_PIVOT = 0.0;

        // feedforward
        public static final double kS_PIVOT = 0.1; // Voltage to overcome static friction
        public static final double kG_PIVOT = 0.4; // Voltage to overcome gravity
        public static final double kV_PIVOT = 0.0; // Velocity gain

        // motion constraints
        public static final double kMinOutput = -1.0;
        public static final double kMaxOutput = 1.0;

        // absolute encoder setup
        // Find this by placing arm at 0 degrees and reading the "Raw" value
        public static final double kEncoderOffset = 0.0; 

        // arm set
        public static final double kAngleStowed = 0.0;   // pushed up
        public static final double kAngleGround = 90.0; // on floor

        // ramp rates
        public static final double kOpenLoopRampRate = 0.5; 
        public static final double kClosedLoopRampRate = 0.0;

        // angles
        public static final double kMinAngle = 0.0;
        public static final double kMaxAngle = 90.0;
    }
  
  public static final class IndexerConstants {
        public static final int INDEXER_MOTOR_ID = 0; // Change to correct port
    
        public static final boolean kIndexerInverted = false;
    
        public static final double kIndexerCurrentLimit = 25.0;
    }
  
}
