// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;

import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkFlex feederRoller;
  private final SparkFlex intakeLauncherRoller;

  /** Creates a new CANFuelSubsystem using shared motors from ShooterSubsystem. */
  public CANFuelSubsystem() {
    // Use the same motor objects as the shooter subsystem (shared CAN IDs 51 & 52)
    intakeLauncherRoller = new SparkFlex(FuelConstants.LEAD_shooterMotorID, MotorType.kBrushless);
    feederRoller = new SparkFlex(FuelConstants.FOLLOW_shooterMotorID, MotorType.kBrushless);
    
    SparkFlexConfig feederConfig = new SparkFlexConfig();
    feederConfig.inverted(true);
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkFlexConfig launcherConfig = new SparkFlexConfig();
    launcherConfig.inverted(false);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the voltage of the intake roller
  public void setIntakeLauncherRoller(double voltage) {
    intakeLauncherRoller.setVoltage(voltage);
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double voltage) {
    feederRoller.setVoltage(voltage);
  }

  // A method to stop the rollers
  public void stop() {
    // Only control leader — follower mirrors it automatically (YAMS hardware follower)
    feederRoller.setVoltage(0);
    intakeLauncherRoller.setVoltage(0);
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

  // Command to launch (feed game piece to shooter)
  // Runs motors while command is active, stops when command ends
  public Command shoot() {
    return this.startEnd(
      () -> {
        setIntakeLauncherRoller(LAUNCHING_LAUNCHER_VOLTAGE);
        setFeederRoller(LAUNCHING_FEEDER_VOLTAGE);
      },
      () -> stop()
    );
  }

  // Command to ferry (lower-power launch for passing)
  // Runs motors while command is active, stops when command ends
  public Command ferry() {
    return this.startEnd(
      () -> {
        setIntakeLauncherRoller(FERRY_LAUNCHER_VOLTAGE);
        setFeederRoller(FERRY_FEEDER_VOLTAGE);
      },
      () -> stop()
    );
  }

    public Command stopCommand() {
    return this.runOnce(() -> {
      // Only set leader motor — follower mirrors it automatically
      stop();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
