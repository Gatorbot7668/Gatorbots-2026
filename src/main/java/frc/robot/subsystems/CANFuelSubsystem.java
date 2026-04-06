// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FuelConstants;
import frc.robot.util.ShooterParameters;

import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkFlex feederRoller;
  private final SparkFlex intakeLauncherRoller;
  private final SparkClosedLoopController launcherPID;
  private final SparkClosedLoopController feederPID;
  private final RelativeEncoder launcherEncoder;
  private final RelativeEncoder feederEncoder;

  // State for adjustingShoot periodic RPM boost
  private double adjustShootBaseRPM = 0;
  private final Timer adjustShootTimer = new Timer();

  /** Creates a new CANFuelSubsystem using shared motors from ShooterSubsystem. */
  public CANFuelSubsystem() {
    // Use the same motor objects as the shooter subsystem (shared CAN IDs 51 & 52)
    intakeLauncherRoller = new SparkFlex(FuelConstants.LEAD_shooterMotorID, MotorType.kBrushless);
    feederRoller = new SparkFlex(FuelConstants.FOLLOW_shooterMotorID, MotorType.kBrushless);

    // --- Feeder config with closed-loop PID ---
    SparkFlexConfig feederConfig = new SparkFlexConfig();
    feederConfig.inverted(true);
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederConfig.closedLoop
        .pid(kFeederP.get(), kFeederI.get(), kFeederD.get())
        .outputRange(-1, 1);
    feederConfig.closedLoop.feedForward
        .kV(kFeederFF.get());
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // --- Launcher config with closed-loop PID ---
    SparkFlexConfig launcherConfig = new SparkFlexConfig();
    launcherConfig.inverted(false);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherConfig.closedLoop
        .pid(kLauncherP.get(), kLauncherI.get(), kLauncherD.get())
        .outputRange(-1, 1);
    launcherConfig.closedLoop.feedForward
        .kV(kLauncherFF.get());
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Grab PID controllers and encoders
    launcherPID = intakeLauncherRoller.getClosedLoopController();
    feederPID = feederRoller.getClosedLoopController();
    launcherEncoder = intakeLauncherRoller.getEncoder();
    feederEncoder = feederRoller.getEncoder();

    // put default values for various fuel operations onto the dashboard
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // --- Velocity helpers ---

  /**
   * Convert a voltage target (0–12V) to an approximate RPM target.
   * Uses the NEO Vortex free speed as the scaling reference.
   */
  private double voltageToRPM(double voltage) {
    return (voltage / 12.0) * NEO_VORTEX_FREE_SPEED_RPM;
  }

  /**
   * Set the launcher motor to a target RPM using closed-loop velocity control.
   */
  public void setLauncherRPM(double rpm) {
    launcherPID.setSetpoint(rpm, ControlType.kVelocity);
  }

  /**
   * Set the feeder motor to a target RPM using closed-loop velocity control.
   */
  public void setFeederRPM(double rpm) {
    feederPID.setSetpoint(rpm, ControlType.kVelocity);
  }

  // A method to set the voltage of the intake roller (open-loop, used for intake only)
  public void setIntakeLauncherRoller(double voltage) {
    intakeLauncherRoller.setVoltage(voltage);
  }

  // A method to set the voltage of the feeder roller (open-loop, used for intake only)
  public void setFeederRoller(double voltage) {
    feederRoller.setVoltage(voltage);
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.setVoltage(0);
    intakeLauncherRoller.setVoltage(0);
  }

  /**
   * Reconfigure the closed-loop PID gains on both motors.
   * Called from RobotContainer.robotPeriodic() when TunableNumbers change.
   */
  public void reconfigurePID() {
    SparkFlexConfig feederUpdate = new SparkFlexConfig();
    feederUpdate.closedLoop
        .pid(kFeederP.get(), kFeederI.get(), kFeederD.get());
    feederUpdate.closedLoop.feedForward
        .kV(kFeederFF.get());
    feederRoller.configure(feederUpdate, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkFlexConfig launcherUpdate = new SparkFlexConfig();
    launcherUpdate.closedLoop
        .pid(kLauncherP.get(), kLauncherI.get(), kLauncherD.get());
    launcherUpdate.closedLoop.feedForward
        .kV(kLauncherFF.get());
    intakeLauncherRoller.configure(launcherUpdate, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // Command to run intake (pull game piece in)
  // Runs motors while command is active, stops when command ends
  public Command intake() {
    return this.startEnd(
      () -> {
        setIntakeLauncherRoller(INTAKING_INTAKE_VOLTAGE);
        setFeederRoller(INTAKING_FEEDER_VOLTAGE);
      },
      () -> stop()
    );
  }

  // Command to reverse intake (push game piece out)
  // Runs motors in opposite direction while command is active, stops when command ends
  public Command reverseIntake() {
    return this.startEnd(
      () -> {
        setIntakeLauncherRoller(-INTAKING_INTAKE_VOLTAGE);
        setFeederRoller(-INTAKING_FEEDER_VOLTAGE);
      },
      () -> stop()
    );
  }



  // Command to launch (spin up launcher motor with PID velocity control)
  // Runs motor while command is active, stops when command ends
  public Command shoot_launcher() {
    return this.startEnd(
      () -> {
        setLauncherRPM(voltageToRPM(LAUNCHING_LAUNCHER_VOLTAGE));
      },
      () -> stop()
    );
  }

  public Command shoot_feeder(){
    return this.startEnd(
    () -> {
      setFeederRPM(voltageToRPM(LAUNCHING_FEEDER_VOLTAGE));
    },
    () -> stop()
    );
  }

  public Command shoot(){
    return new SequentialCommandGroup(
      new InstantCommand(() -> setLauncherRPM(voltageToRPM(LAUNCHING_LAUNCHER_VOLTAGE))),
      new WaitCommand(1),
      new InstantCommand(() -> setFeederRPM(voltageToRPM(LAUNCHING_FEEDER_VOLTAGE))),
      new WaitCommand(3),
      new InstantCommand(() -> stop())
    );
  }

  public Command maxShoot(){
    return this.startEnd(
    () -> {
      setLauncherRPM(voltageToRPM(MAXIMUM_VOLTAGE));
      setFeederRPM(voltageToRPM(LAUNCHING_FEEDER_VOLTAGE));
    },
    () -> stop()
    );
  }

  /**
   * Adjusts shoot RPM based on distance to target (using Limelight ta),
   * using a lookup table for empirically-tested RPM values.
   *
   * ta is the target area as a percentage of the camera frame:
   *   - Large ta = close to target = less RPM needed
   *   - Small ta = far from target = more RPM needed
   *
   * The lookup table (SHOOTER_LOOKUP_TABLE in Constants) maps ta values
   * to tested RPM values and interpolates between sample points.
   *
   * Uses closed-loop velocity PID so motors maintain speed even under load.
   * Launcher spins up first, then feeder starts after 2 seconds.
   * While held, the target RPM ramps up by ADJUSTING_SHOOT_RPM_BOOST_PER_SECOND
   * every second to fight droop. Release the button to stop all motors.
   *
   * @param vision The VisionSubsystem to read ta from
   */
  public Command adjustingShoot(VisionSubsystem vision) {
    return new SequentialCommandGroup(
      // Step 1: Read ta and lookup RPM from table, then start launcher only.
      //         Also reset the boost timer.
      new InstantCommand(() -> {
        double ta = vision.get_ta();
        double clampedTa = MathUtil.clamp(ta, ADJUSTING_SHOOT_TA_MIN, ADJUSTING_SHOOT_TA_MAX);
        
        // LOOKUP TABLE: Get interpolated parameters for this distance
        ShooterParameters params = SHOOTER_LOOKUP_TABLE.get(clampedTa);
        
        adjustShootBaseRPM = params.launcherRPM;
        adjustShootTimer.restart();   // reset and start the boost timer

        SmartDashboard.putNumber("AdjustingShoot/ta", ta);
        SmartDashboard.putNumber("AdjustingShoot/clampedTa", clampedTa);
        SmartDashboard.putNumber("AdjustingShoot/launcherRPM", params.launcherRPM);
        SmartDashboard.putNumber("AdjustingShoot/feederRPM", params.feederRPM);

        setLauncherRPM(params.launcherRPM);
      }),
      // Step 2: Wait 2 seconds for launcher to spin up
      new WaitCommand(2),
      // Step 3: Start feeder using lookup table (re-read ta for latest value)
      new InstantCommand(() -> {
        double ta = vision.get_ta();
        double clampedTa = MathUtil.clamp(ta, ADJUSTING_SHOOT_TA_MIN, ADJUSTING_SHOOT_TA_MAX);
        
        // LOOKUP TABLE: Get interpolated parameters for this distance
        ShooterParameters params = SHOOTER_LOOKUP_TABLE.get(clampedTa);

        // Update base RPM to this value (in case ta changed during spin-up)
        adjustShootBaseRPM = params.launcherRPM;
        adjustShootTimer.restart();   // restart timer from feeder-start moment

        SmartDashboard.putNumber("AdjustingShoot/ta", ta);
        SmartDashboard.putNumber("AdjustingShoot/launcherRPM", params.launcherRPM);
        SmartDashboard.putNumber("AdjustingShoot/feederRPM", params.feederRPM);

        setFeederRPM(params.feederRPM);
      }),
      // Step 4: Periodically re-apply RPM with a boost that increases over time.
      //         Every 20ms loop iteration, the target RPM = baseRPM + (elapsed seconds * boost/sec).
      //         Capped at max RPM (NEO Vortex free speed) so we don't exceed motor limits.
      Commands.run(() -> {
        double elapsed = adjustShootTimer.get();
        double boost = elapsed * ADJUSTING_SHOOT_RPM_BOOST_PER_SECOND;
        double boostedRPM = Math.min(adjustShootBaseRPM + boost, NEO_VORTEX_FREE_SPEED_RPM);

        SmartDashboard.putNumber("AdjustingShoot/boostedRPM", boostedRPM);

        setLauncherRPM(boostedRPM);
        setFeederRPM(boostedRPM);
      })
    ).finallyDo(() -> {
      adjustShootTimer.stop();
      stop();
    });
  }


    

  // Command to ferry (lower-power launch for passing)
  // Spins up launcher for 2 seconds, then feeds. Stops when released.
  public Command ferry() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        setLauncherRPM(voltageToRPM(FERRY_FEEDER_VOLTAGE));
      }),
      new WaitCommand(2),
      new InstantCommand(() -> {
        setFeederRPM(voltageToRPM(FERRY_FEEDER_VOLTAGE));
      }),
      Commands.run(() -> {})
    ).finallyDo(() -> stop());
  }

    public Command stopCommand() {
    return this.runOnce(() -> {
      // Only set leader motor — follower mirrors it automatically
      stop();
    });
  }

  @Override
  public void periodic() {
    // Display actual motor velocities for PID tuning
    SmartDashboard.putNumber("Shooter/Launcher/actualRPM", launcherEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter/Feeder/actualRPM", feederEncoder.getVelocity());
  }
}
