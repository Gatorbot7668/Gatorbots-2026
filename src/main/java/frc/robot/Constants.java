// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.InterpolatingShooterMap;
import frc.robot.util.ShooterParameters;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.TunableNumber;
import swervelib.math.Matter;

public final class Constants
{
  public static record TwoIDs(int id1, int id2) {}

 
  public static final class CANDeviceID {
    // CAN IDs 1-8 are taken by Swerve motors and 9-12 - by Swerve encoders

    public static final int kMotor = 17;

  }

  public static final class DIOPort {
    public static final int kDutyEncoder = 0;
    public static final int kQuadratureEncoderChannelA = 1;
    public static final int kQuadratureEncoderChannelB = 2;
  }

  public static class SwerveConstants {
    // see also PID constants in pidfproperties.json

    // Maximum speed of the robot in meters per second, used to limit acceleration.
    public final static LinearVelocity kMaxSpeed = MetersPerSecond.of(4);

    // Drive motor feedforward gains (from SysId characterization).
    // TODO: Run SysId on the real robot and replace these placeholder values!
    //   Use driveCharacterizationSysIdCommand(), then plug in the results here.
    public static final TunableNumber kDriveS = new TunableNumber("Swerve/FF/kS", 0.0);   // Volts to overcome static friction
    public static final TunableNumber kDriveV = new TunableNumber("Swerve/FF/kV", 0.0);   // Volts per meter-per-second
    public static final TunableNumber kDriveA = new TunableNumber("Swerve/FF/kA", 0.0);   // Volts per meter-per-second-squared
  }
  //This means the drive motors get no feedforward assistance — the PID has to do all the work from scratch, making it sluggish and inaccurate.

  public static final class PathPlannerConstants {
    public static final PIDConstants kTranslationPID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants kAnglePID = new PIDConstants(0.4, 0, 0.01);
    public final static LinearVelocity kMaxSpeed = MetersPerSecond.of(4);
  }

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final Time kWheelLockTime = Seconds.of(10);
  }

  public static final class FuelConstants {
    // Motor CAN IDs are now in ShooterConstants (shared motors, CAN 51 & 52)

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LEAD_shooterMotorID = 52; // RIGHT
    public static final int FOLLOW_shooterMotorID = 51; // LEFT
    public static final int EXTRA_HOPPER_MOTOR_ID = 53; // Extra motor in the hopper, follows 51 (same direction, at the same time)

    // NEO Vortex free speed is ~6784 RPM. We use this to convert voltage targets to RPM targets.
    public static final double NEO_VORTEX_FREE_SPEED_RPM = 6784.0;

    // Shared PID gains for both launcher AND feeder velocity control (tunable via SmartDashboard)
    // Both motors use the same PID values since they are the same motor type (NEO Vortex)
    public static final TunableNumber kShooterP  = new TunableNumber("Shooter/PID/kP", 0.05);
    public static final TunableNumber kShooterI  = new TunableNumber("Shooter/PID/kI", 0.0);
    public static final TunableNumber kShooterD  = new TunableNumber("Shooter/PID/kD", 0.0);

    // PID + FF gains for hopper velocity control (tunable via SmartDashboard)
    public static final TunableNumber kHopperP  = new TunableNumber("Shooter/Hopper/kP", 0.05);
    public static final TunableNumber kHopperI  = new TunableNumber("Shooter/Hopper/kI", 0.0);
    public static final TunableNumber kHopperD  = new TunableNumber("Shooter/Hopper/kD", 0.0);
    public static final TunableNumber kHopperFF = new TunableNumber("Shooter/Hopper/kFF", 1.0 / NEO_VORTEX_FREE_SPEED_RPM);

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = -5;
    public static final double INTAKING_INTAKE_VOLTAGE = 5;
    public static final double INTAKING_HOPPER_VOLTAGE = -5;  // Hopper roller voltage during intake (same direction as feeder)
    public static final double LAUNCHING_FEEDER_VOLTAGE = 11;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 11;
    public static final double LAUNCHING_HOPPER_VOLTAGE = 5;
    public static final double FERRY_FEEDER_VOLTAGE = 4;
    public static final double FERRY_LAUNCHER_VOLTAGE = 5;
    public static final double SPIN_UP_FEEDER_VOLTAGE = -6;
    public static final double SPIN_UP_SECONDS = 1.5;
    public static final double MAXIMUM_VOLTAGE = 12;

    // Adjusting shoot constants
    // ta range: the expected min/max target areas for the lookup table
    // Small ta = far from target (needs more RPM); Large ta = close (needs less RPM)
    public static final double ADJUSTING_SHOOT_TA_MIN = 0.05;   // very far (tiny target in frame)
    public static final double ADJUSTING_SHOOT_TA_MAX = 6.0;   // very close (large target in frame)

    // RPM boost applied periodically during adjustingShoot to compensate for voltage droop.
    // The target RPM increases by this amount every second while the button is held.
    // TODO: Tune this value on the real robot!
    public static final double ADJUSTING_SHOOT_RPM_BOOST_PER_SECOND = 10.0;

    // Tunable RPM values for testShooterRPM() - adjust these on SmartDashboard during testing
    public static final TunableNumber kTestLauncherRPM = new TunableNumber("ShooterTest/launcherTargetRPM", 3000);
    public static final TunableNumber kTestFeederRPM = new TunableNumber("ShooterTest/feederTargetRPM", 3000);

    // Tunable RPM range for adjustingShootSimple() — maps ta directly to RPM without lookup table
    // At ta = TA_MIN (very far), RPM = maxRPM. At ta = TA_MAX (very close), RPM = minRPM.
    public static final TunableNumber kSimpleShootMaxRPM = new TunableNumber("SimpleShoot/maxRPM", 6000);  // RPM when far away (small ta)
    public static final TunableNumber kSimpleShootMinRPM = new TunableNumber("SimpleShoot/minRPM", 2000);  // RPM when close (large ta)

    /**
     * Lookup table: Target Area (ta) -> Shooter Parameters (RPM)
     * 
     * ta = how large the AprilTag appears in the Limelight frame (percentage).
     * Smaller ta = farther away = higher RPM needed.
     * Larger ta = closer = lower RPM needed.
     * 
     * HOW TO TUNE:
     * 1. Position robot at a specific spot facing the target
     * 2. Note the "Vision/ta_shooter" value shown on SmartDashboard
     * 3. Manually test different RPM values until shots consistently score
     * 4. Add that (ta, RPM) pair below using addSample()
     * 5. Repeat at 10-15 different positions for good interpolation
     * 
     * IMPORTANT: ta values are INVERSELY related to distance:
     *   - ta ≈ 0.1-0.5  = very far from target  = high RPM
     *   - ta ≈ 1.0-2.0  = medium distance        = medium RPM
     *   - ta ≈ 3.0-6.0  = very close to target   = low RPM
     */
    public static final InterpolatingShooterMap SHOOTER_LOOKUP_TABLE = new InterpolatingShooterMap();

    static {
      // Format: SHOOTER_LOOKUP_TABLE.addSample(ta_percentage, new ShooterParameters(launcherRPM, feederRPM));
      // Smaller ta = farther away = higher RPM needed.
      // Larger  ta = closer       = lower  RPM needed.
      //
      // These are ESTIMATED starting values — tune each point on the real robot
      // using testShooterRPM() and reading "Vision/ta_shooter" on SmartDashboard.

      SHOOTER_LOOKUP_TABLE.addSample(0.05, new ShooterParameters(6784.0, 6784.0));  // Extremely far — full send
      SHOOTER_LOOKUP_TABLE.addSample(0.10, new ShooterParameters(6600.0, 6600.0));  // Very far
      SHOOTER_LOOKUP_TABLE.addSample(0.20, new ShooterParameters(6400.0, 6400.0));
      SHOOTER_LOOKUP_TABLE.addSample(0.35, new ShooterParameters(6100.0, 6100.0));
      SHOOTER_LOOKUP_TABLE.addSample(0.50, new ShooterParameters(5800.0, 5800.0));
      SHOOTER_LOOKUP_TABLE.addSample(0.70, new ShooterParameters(5500.0, 5500.0));
      SHOOTER_LOOKUP_TABLE.addSample(0.90, new ShooterParameters(5200.0, 5200.0));
      SHOOTER_LOOKUP_TABLE.addSample(1.10, new ShooterParameters(4900.0, 4900.0));
      SHOOTER_LOOKUP_TABLE.addSample(1.30, new ShooterParameters(4650.0, 4650.0));
      SHOOTER_LOOKUP_TABLE.addSample(1.60, new ShooterParameters(4400.0, 4400.0));
      SHOOTER_LOOKUP_TABLE.addSample(1.90, new ShooterParameters(4150.0, 4150.0));
      SHOOTER_LOOKUP_TABLE.addSample(2.20, new ShooterParameters(3900.0, 3900.0));
      SHOOTER_LOOKUP_TABLE.addSample(2.60, new ShooterParameters(3650.0, 3650.0));
      SHOOTER_LOOKUP_TABLE.addSample(3.00, new ShooterParameters(3400.0, 3400.0));
      SHOOTER_LOOKUP_TABLE.addSample(3.50, new ShooterParameters(3100.0, 3100.0));
      SHOOTER_LOOKUP_TABLE.addSample(4.00, new ShooterParameters(2850.0, 2850.0));
      SHOOTER_LOOKUP_TABLE.addSample(4.50, new ShooterParameters(2600.0, 2600.0));
      SHOOTER_LOOKUP_TABLE.addSample(5.00, new ShooterParameters(2400.0, 2400.0));
      SHOOTER_LOOKUP_TABLE.addSample(5.50, new ShooterParameters(2200.0, 2200.0));
      SHOOTER_LOOKUP_TABLE.addSample(6.00, new ShooterParameters(2000.0, 2000.0));  // Very close — gentle lob
    }
  }


  /// PROBLEM: Fuelsubsystem and Shootersubsystem share the same physical motors... right now when we deploy both of them does not perform the job.
  /// Conflict. 

  public static final class VisionConstants {
    // Limelight pipeline indices — must match what's configured in the Limelight web UI
    public static final int PIPELINE_APRILTAG = 0;
    public static final int PIPELINE_RETROREFLECTIVE = 1;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondaryDriverControllerPort = 1;

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double RIGHT_Y_DEADBAND = 0.1;
  }

  public static class AdvancedDriveCommandsConstants {
    public static final Mass kRobotMass = Pounds.of(50);
    public static final Matter kChassisMatter = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), kRobotMass.in(Kilograms));
    public static final Time kLooptime = Milliseconds.of(130); // 20ms + 110ms sprk max velocity lag
    public static final AngularVelocity kTurnSpeed = RotationsPerSecond.of(1);
  }

  // Whether TunableNumbers are changeable via SmartDashboard
  public static final boolean kTuningMode = true;
}
