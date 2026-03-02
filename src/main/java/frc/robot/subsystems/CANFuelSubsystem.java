// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  // ! it's lowkey a better practice to define your motor configs statically and at the top of your file
  // create the configuration for the feeder roller, set a current limit
  public static final SparkBaseConfig kFeederConfig = 
    new SparkFlexConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT)
      .voltageCompensation(12.0);

  // create the configuration for the launcher roller, set a current limit, set
  // the motor to inverted so that positive values are used for both intaking and
  // launching, and apply the config to the controller
  public static final SparkBaseConfig kLauncherConfig = 
    new SparkFlexConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT)
      .voltageCompensation(12.0)
      .inverted(true);

  private final SparkFlex feederRoller;
  private final SparkFlex intakeLauncherRoller;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new SparkFlex(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkFlex(FEEDER_MOTOR_ID, MotorType.kBrushless);

    /**
     * ! Hi the reason all of these are decpreated is cuz you have to use
     * ! the new resetmode/persistmode classees or whatever
     */
    // Apply configs
    feederRoller.configure(kFeederConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Wait thIS SADIE DONT FORGET guys it says to change the kresetsafe parameters BECAUSE ITS DEPRECATED AND MIGHT GET REMOVED IN FUTURE UPDATES, BUT IM TOO SCARED TO MESS ANYTHING UP SO WHEN YOU GUYS GET HERE LETS LOOK THIS OVER
    // ! Young one, you must never fear to get messy. Remember: If anything goes wrong, blame it all on electrical
    intakeLauncherRoller.configure(kLauncherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);

    // ! instead of using .finallyDo() just set a stop command as subsystem's default command (command to run when not running any other commands)
    setDefaultCommand(stop());
  }
  
  // ! rewrote somethings below in full command base for ease

  /** Set voltage setpoints for the feeder & launcher motors */
  public Command setVoltages(double feederVoltage, double launcherVoltage) {
    return this.run(() -> {
      feederRoller.setVoltage(feederVoltage);
      intakeLauncherRoller.setVoltage(launcherVoltage);
    });
  }

  /** Stop all motors */
  public Command stop() {
    return this.run(() -> {
      feederRoller.stopMotor();
      intakeLauncherRoller.stopMotor();
    });
  }

  // Command to run intake (pull game piece in)
  public Command intake() {
    return setVoltages(INTAKING_FEEDER_VOLTAGE, INTAKING_INTAKE_VOLTAGE);
  }

  // Command to launch (feed game piece to shooter)
  public Command launch() {
    return setVoltages(LAUNCHING_FEEDER_VOLTAGE, LAUNCHING_LAUNCHER_VOLTAGE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
