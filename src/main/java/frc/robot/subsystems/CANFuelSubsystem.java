// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
  private final SparkFlex extraHopperRoller;
  private final RelativeEncoder launcherEncoder;
  private final RelativeEncoder feederEncoder;
  private final RelativeEncoder hopperEncoder;

  // WPILib software PID controllers (run on the roboRIO in periodic())
  private final PIDController launcherPIDController;
  private final PIDController feederPIDController;
  private final PIDController hopperPIDController;

  // Target RPM for each motor (0 = stopped / open-loop mode)
  private double launcherTargetRPM = 0;
  private double feederTargetRPM = 0;
  private double hopperTargetRPM = 0;

  // When true, periodic() runs the PID loop for that motor.
  // When false, the motor is in open-loop voltage mode (e.g., during intake or stopped).
  private boolean launcherPIDEnabled = false;
  private boolean feederPIDEnabled = false;
  private boolean hopperPIDEnabled = false;

  /** Creates a new CANFuelSubsystem using shared motors from ShooterSubsystem. */
  public CANFuelSubsystem() {
    // Use the same motor objects as the shooter subsystem (shared CAN IDs 51 & 52)
    intakeLauncherRoller = new SparkFlex(FuelConstants.LEAD_shooterMotorID, MotorType.kBrushless);
    feederRoller = new SparkFlex(FuelConstants.FOLLOW_shooterMotorID, MotorType.kBrushless);

    // --- Feeder config (NO closed-loop PID on SparkFlex — PID runs on roboRIO) ---
    SparkFlexConfig feederConfig = new SparkFlexConfig();
    feederConfig.inverted(true);
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // --- Extra hopper motor (CAN 53) ---
    extraHopperRoller = new SparkFlex(FuelConstants.EXTRA_HOPPER_MOTOR_ID, MotorType.kBrushless);
    SparkFlexConfig hoppermotorConfig = new SparkFlexConfig();
    hoppermotorConfig.inverted(true); // same direction as feeder (CAN 51)
    hoppermotorConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    extraHopperRoller.configure(hoppermotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // --- Launcher config ---
    SparkFlexConfig launcherConfig = new SparkFlexConfig();
    launcherConfig.inverted(false);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Grab encoders (for reading actual RPM in periodic())
    launcherEncoder = intakeLauncherRoller.getEncoder();
    feederEncoder = feederRoller.getEncoder();
    hopperEncoder = extraHopperRoller.getEncoder();

    // Create WPILib PID controllers for each motor
    // These run on the roboRIO at 50Hz (every 20ms) in periodic()
    // Launcher and feeder share the same PID gains (kShooterP/I/D)
    launcherPIDController = new PIDController(kShooterP.get(), kShooterI.get(), kShooterD.get());
    feederPIDController   = new PIDController(kShooterP.get(), kShooterI.get(), kShooterD.get());
    hopperPIDController   = new PIDController(kHopperP.get(), kHopperI.get(), kHopperD.get());

    // Allow continuous wrapping is not needed for velocity — PID will just track RPM

    // put default values for various fuel operations onto the dashboard
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // --- Velocity helpers ---

  /**
   * Convert a voltage target (0-12V) to an approximate RPM target.
   * Uses the NEO Vortex free speed as the scaling reference.
   */
  private double voltageToRPM(double voltage) {
    return (voltage / 12.0) * NEO_VORTEX_FREE_SPEED_RPM;
  }

  /**
   * Set the launcher motor to a target RPM using software PID (runs in periodic()).
   * The PID loop in periodic() will continuously adjust voltage to reach this RPM.
   */
  public void setLauncherRPM(double rpm) {
    launcherTargetRPM = rpm;
    launcherPIDEnabled = true;
    launcherPIDController.reset(); // clear accumulated I-term from previous run
  }

  /**
   * Set the feeder motor to a target RPM using software PID (runs in periodic()).
   */
  public void setFeederRPM(double rpm) {
    feederTargetRPM = rpm;
    feederPIDEnabled = true;
    feederPIDController.reset();
  }

  /**
   * Set the hopper motor to a target RPM using software PID (runs in periodic()).
   */
  public void setHopperRPM(double rpm) {
    hopperTargetRPM = rpm;
    hopperPIDEnabled = true;
    hopperPIDController.reset();
  }

  

  // A method to set the voltage of the intake roller (open-loop, used for intake only)
  public void setIntakeLauncherRoller(double voltage) {
    intakeLauncherRoller.setVoltage(voltage);
  }

  // A method to set the voltage of the feeder roller (open-loop, used for intake only)
  public void setFeederRoller(double voltage) {
    feederRoller.setVoltage(voltage);
  }

  // A method to set the voltage of the hopper roller (open-loop, CAN 53)
  public void setHopperRoller(double voltage) {
    extraHopperRoller.setVoltage(voltage);
  }

  // A method to stop the rollers and disable PID
  public void stop() {
    // Disable PID so periodic() stops driving the motors
    launcherPIDEnabled = false;
    feederPIDEnabled = false;
    hopperPIDEnabled = false;
    launcherTargetRPM = 0;
    feederTargetRPM = 0;
    hopperTargetRPM = 0;
    // Send 0V to all motors
    feederRoller.setVoltage(0);
    intakeLauncherRoller.setVoltage(0);
    extraHopperRoller.setVoltage(0);
  }

  /**
   * Reconfigure the WPILib PID gains on all three motors.
   * Called from RobotContainer.robotPeriodic() when TunableNumbers change.
   */
  public void reconfigurePID() {
    launcherPIDController.setPID(kShooterP.get(), kShooterI.get(), kShooterD.get());
    feederPIDController.setPID(kShooterP.get(), kShooterI.get(), kShooterD.get());
    hopperPIDController.setPID(kHopperP.get(), kHopperI.get(), kHopperD.get());
  }

  // Command to run intake (pull game piece in)
  // Runs motors while command is active, stops when command ends
  public Command intake() {
    return this.startEnd(
      () -> {
        //setLauncherRPM(voltageToRPM(INTAKING_INTAKE_VOLTAGE)); // use closed-loop velocity control for intake roller to maintain consistent speed under load
        //setFeederRPM(voltageToRPM(INTAKING_FEEDER_VOLTAGE));
        //setHopperRPM(voltageToRPM(INTAKING_HOPPER_VOLTAGE));
        setIntakeLauncherRoller(INTAKING_INTAKE_VOLTAGE);
        setFeederRoller(INTAKING_FEEDER_VOLTAGE);
        setHopperRoller(INTAKING_HOPPER_VOLTAGE);
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
        setHopperRoller(-INTAKING_HOPPER_VOLTAGE);
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
      setHopperRPM(voltageToRPM(LAUNCHING_HOPPER_VOLTAGE));
    },
    () -> stop()
    );
  }

  public Command shoot(){
    return new SequentialCommandGroup(
      new InstantCommand(() -> setLauncherRPM(voltageToRPM(LAUNCHING_LAUNCHER_VOLTAGE))),
      new WaitCommand(1),
      new InstantCommand(() -> {
        setFeederRPM(voltageToRPM(LAUNCHING_FEEDER_VOLTAGE));
        setHopperRPM(voltageToRPM(LAUNCHING_HOPPER_VOLTAGE));
      }),
      new WaitCommand(3),
      new InstantCommand(() -> stop())
    );
  }

  public Command maxShoot(){
    return new SequentialCommandGroup(
      // Step 1: Spin up launcher at full voltage
      new InstantCommand(() -> setIntakeLauncherRoller(MAXIMUM_VOLTAGE)),
      // Step 2: Wait 2 seconds for spin-up
      new WaitCommand(2),
      // Step 3: Start feeder + hopper, keep launcher running
      Commands.run(() -> {
        setIntakeLauncherRoller(MAXIMUM_VOLTAGE);
        setFeederRoller(MAXIMUM_VOLTAGE);
        setHopperRoller(LAUNCHING_HOPPER_VOLTAGE);
      })
    ).finallyDo(() -> stop());
  }

  /**
   * ===========================================================================
   * TESTING COMMAND: Use this to find the RPM values for your lookup table!
   * ===========================================================================
   * 
   * HOW TO USE:
   * 1. Add these TunableNumbers to SmartDashboard:
   *    - "ShooterTest/launcherTargetRPM" (editable, default 3000)
   *    - "ShooterTest/feederTargetRPM" (editable, default 3000)
   * 2. Position robot at a specific position facing the target
   * 3. Note the "ShooterTest/currentTA" value shown on SmartDashboard
   * 4. Hold the button to spin up and shoot
   * 5. Adjust target RPM values until shots consistently score
   * 6. Record: ta = X, launcherRPM = Y, feederRPM = Z
   * 7. Move to a new position and repeat
   * 8. After testing 10-15 positions, add values to SHOOTER_LOOKUP_TABLE in Constants.java
   * 
   * @param vision The VisionSubsystem to read ta from (for display only)
   */
  public Command testShooterRPM(VisionSubsystem vision) {
    return new SequentialCommandGroup(
      // Step 1: Show current ta and start launcher at manual RPM
      new InstantCommand(() -> {
        double ta = vision.getTargetAreaForShooter();
        double launcherTargetRPM = kTestLauncherRPM.get();
        
        SmartDashboard.putNumber("ShooterTest/currentTA", ta);
        
        // Start launcher only
        setIntakeLauncherRoller(rpmToVoltage(launcherTargetRPM));
        //setLauncherRPM(launcherTargetRPM);
      }),

      
      // Step 2: Wait 2 seconds for spin-up
      new WaitCommand(2),
      
      // Step 3: Start feeder and keep running, continuously show ta and actual RPM
      Commands.run(() -> {
        double ta = vision.getTargetAreaForShooter();
        double launcherTargetRPM = kTestLauncherRPM.get();
        double feederTargetRPM = kTestFeederRPM.get();
        
        // Display current values for recording
        SmartDashboard.putNumber("ShooterTest/currentTA", ta);
        SmartDashboard.putNumber("ShooterTest/launcherActualRPM", launcherEncoder.getVelocity());
        SmartDashboard.putNumber("ShooterTest/feederActualRPM", feederEncoder.getVelocity());
        
        // Run motors at their respective target RPMs (reads from TunableNumber each loop)
        //setLauncherRPM(launcherTargetRPM);
        setIntakeLauncherRoller(rpmToVoltage(launcherTargetRPM));
        setFeederRoller(rpmToVoltage(feederTargetRPM));
        //setFeederRPM(feederTargetRPM);
      })
    ).finallyDo(() -> stop());
  }

  /**
   * Adjusts shoot RPM based on target area (ta) from the Limelight,
   * using a lookup table for empirically-tested RPM values.
   *
   * ta = how large the AprilTag appears in the camera frame:
   *   - Small ta = far from target = more RPM needed
   *   - Large ta = close to target = less RPM needed
   *
   * ta naturally accounts for both distance AND angle to the target.
   * When the robot is diagonal to the hub, ta is smaller because the tag
   * appears narrower, AND the straight-line distance is longer — so higher
   * RPM is the correct response.
   *
   * The lookup table (SHOOTER_LOOKUP_TABLE in Constants) maps ta values
   * to tested RPM values and interpolates between sample points.
   *
   * Uses closed-loop velocity PID (WPILib PIDController in periodic())
   * so motors maintain speed even under load.
   *
   * Sequence:
   *   1. Read ta, lookup RPM, start launcher
   *   2. Wait 2 seconds for launcher to spin up
   *   3. Start feeder, continuously update both based on current ta
   *   4. When button released, stop all motors
   *
   * @param vision The VisionSubsystem to read ta from
   */
  public Command adjustingShoot(VisionSubsystem vision) {
    return new SequentialCommandGroup(
      // Step 1: Read ta and lookup RPM from table, then start launcher only.
      new InstantCommand(() -> {
        double ta = vision.getTargetAreaForShooter();
        double clampedTA = MathUtil.clamp(ta, ADJUSTING_SHOOT_TA_MIN, ADJUSTING_SHOOT_TA_MAX);
        
        // LOOKUP TABLE: Get interpolated parameters for this ta
        ShooterParameters params = SHOOTER_LOOKUP_TABLE.get(clampedTA);

        SmartDashboard.putNumber("AdjustingShoot/ta", ta);
        SmartDashboard.putNumber("AdjustingShoot/launcherRPM", params.launcherRPM);

        // Start launcher only - PID maintains speed automatically
        //setLauncherRPM(params.launcherRPM);
        setIntakeLauncherRoller(rpmToVoltage(params.launcherRPM));
      }),
      
      // Step 2: Wait 2 seconds for launcher to spin up
      new WaitCommand(2),
      
      // Step 3: Start feeder and keep both running until button released.
      //         Continuously re-reads ta so RPM adjusts if robot moves.
      Commands.run(() -> {
        double ta = vision.getTargetAreaForShooter();
        double clampedTA = MathUtil.clamp(ta, ADJUSTING_SHOOT_TA_MIN, ADJUSTING_SHOOT_TA_MAX);
        
        // LOOKUP TABLE: Get interpolated parameters for current ta
        ShooterParameters params = SHOOTER_LOOKUP_TABLE.get(clampedTA);

        SmartDashboard.putNumber("AdjustingShoot/ta", ta);
        SmartDashboard.putNumber("AdjustingShoot/launcherRPM", params.launcherRPM);
        SmartDashboard.putNumber("AdjustingShoot/feederRPM", params.feederRPM);

        // PID maintains these speeds automatically
        setIntakeLauncherRoller(rpmToVoltage(params.launcherRPM));
        setFeederRoller(rpmToVoltage(params.feederRPM));
        setHopperRoller(LAUNCHING_HOPPER_VOLTAGE);
      })
    ).finallyDo(() -> stop());
  }

  /**
   * Adjusting shoot WITHOUT lookup table or PID — uses a simple formula to convert ta to RPM,
   * then converts that RPM directly to voltage and sends it raw to the motors (open-loop).
   * 
   * Formula: RPM = maxRPM / (1 + k * ta)
   * Voltage = (RPM / 6784) * 12V
   * 
   * If no target is visible, motors don't spin.
   * maxRPM and minRPM are tunable on SmartDashboard under "SimpleShoot/".
   * 
   * @param vision The VisionSubsystem to read ta from
   */
  public Command adjustingShootSimple(VisionSubsystem vision) {
    return new SequentialCommandGroup(
      // Step 1: Read ta, calculate voltage, start launcher only (skip if no target)
      new InstantCommand(() -> {
        if (!vision.hasTarget()) {
          SmartDashboard.putString("SimpleShoot/status", "NO TARGET");
          return;
        }
        double ta = vision.getTargetAreaForShooter();
        double rpm = taToRPM(ta);
        double voltage = rpmToVoltage(rpm);
        
        SmartDashboard.putString("SimpleShoot/status", "SPINNING UP");
        SmartDashboard.putNumber("SimpleShoot/ta", ta);
        SmartDashboard.putNumber("SimpleShoot/targetRPM", rpm);
        SmartDashboard.putNumber("SimpleShoot/voltage", voltage);

        intakeLauncherRoller.setVoltage(voltage);
      }),
      
      // Step 2: Wait 2 seconds for spin-up
      new WaitCommand(2),
      
      // Step 3: Start feeder + hopper, continuously update from ta
      Commands.run(() -> {
        if (!vision.hasTarget()) {
          SmartDashboard.putString("SimpleShoot/status", "NO TARGET - HOLDING");
          return;
        }
        double ta = vision.getTargetAreaForShooter();
        double rpm = taToRPM(ta);
        double voltage = rpmToVoltage(rpm);
        
        SmartDashboard.putString("SimpleShoot/status", "SHOOTING");
        SmartDashboard.putNumber("SimpleShoot/ta", ta);
        SmartDashboard.putNumber("SimpleShoot/targetRPM", rpm);
        SmartDashboard.putNumber("SimpleShoot/voltage", voltage);
        SmartDashboard.putNumber("SimpleShoot/launcherActualRPM", launcherEncoder.getVelocity());
        SmartDashboard.putNumber("SimpleShoot/feederActualRPM", feederEncoder.getVelocity());
        
        // Raw voltage — no PID
        intakeLauncherRoller.setVoltage(voltage);
        feederRoller.setVoltage(voltage);
        extraHopperRoller.setVoltage(rpmToVoltage(voltageToRPM(LAUNCHING_HOPPER_VOLTAGE)));
      })
    ).finallyDo(() -> {
      // Stop all motors directly (no PID to disable)
      intakeLauncherRoller.setVoltage(0);
      feederRoller.setVoltage(0);
      extraHopperRoller.setVoltage(0);
    });
  }

  /**
   * Convert RPM to voltage (inverse of voltageToRPM).
   * voltage = (rpm / freeSpeedRPM) * 12V
   */
  private double rpmToVoltage(double rpm) {
    return (rpm / NEO_VORTEX_FREE_SPEED_RPM) * 12.0;
  }

  /**
   * Convert ta (target area percentage) directly to RPM.
   * Uses an inverse formula so that:
   *   - Small ta (far away) → high RPM
   *   - Large ta (close) → low RPM
   * 
   * Formula: RPM = maxRPM / (1 + k * ta),  where k = (maxRPM/minRPM - 1)
   * No clamping — works with whatever ta the Limelight returns.
   */
  private double taToRPM(double ta) {
    double maxRPM = FuelConstants.kSimpleShootMaxRPM.get();
    double minRPM = FuelConstants.kSimpleShootMinRPM.get();
    
    // k scales so that at ta=1.0, RPM = minRPM
    double k = (maxRPM / minRPM) - 1.0;
    return maxRPM / (1.0 + k * ta);
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
        setHopperRPM(voltageToRPM(LAUNCHING_HOPPER_VOLTAGE));
      }),
      Commands.run(() -> {})
    ).finallyDo(() -> stop());
  }

    public Command stopCommand() {
    return this.runOnce(() -> {
      stop();
    });
  }


  @Override
  public void periodic() {
    // Read actual RPM from encoders
    double launcherActualRPM = launcherEncoder.getVelocity();
    double feederActualRPM = feederEncoder.getVelocity();
    double hopperActualRPM = hopperEncoder.getVelocity();

    // --- LAUNCHER PID LOOP ---
    if (launcherPIDEnabled) {
      // Feedforward: estimate the base voltage needed for the target RPM
      // voltage = (targetRPM / freeSpeedRPM) * 12V
      double launcherFF = (launcherTargetRPM / NEO_VORTEX_FREE_SPEED_RPM) * 12.0;
      // PID correction: adjust based on error (target - actual)
      double launcherPIDOutput = launcherPIDController.calculate(launcherActualRPM, launcherTargetRPM);
      // Total voltage = feedforward + PID correction, clamped to [-12, 12]
      double launcherVoltage = MathUtil.clamp(launcherFF + launcherPIDOutput, -12.0, 12.0);
      intakeLauncherRoller.setVoltage(launcherVoltage);

      // Debug: show what the PID is doing
      SmartDashboard.putNumber("Shooter/Launcher/targetRPM", launcherTargetRPM);
      SmartDashboard.putNumber("Shooter/Launcher/errorRPM", launcherTargetRPM - launcherActualRPM);
      SmartDashboard.putNumber("Shooter/Launcher/ff", launcherFF);
      SmartDashboard.putNumber("Shooter/Launcher/pidOutput", launcherPIDOutput);
      SmartDashboard.putNumber("Shooter/Launcher/totalVoltage", launcherVoltage);
    }

    // --- FEEDER PID LOOP ---
    if (feederPIDEnabled) {
      double feederFF = (feederTargetRPM / NEO_VORTEX_FREE_SPEED_RPM) * 12.0;
      double feederPIDOutput = feederPIDController.calculate(feederActualRPM, feederTargetRPM);
      double feederVoltage = MathUtil.clamp(feederFF + feederPIDOutput, -12.0, 12.0);
      feederRoller.setVoltage(feederVoltage);

      SmartDashboard.putNumber("Shooter/Feeder/targetRPM", feederTargetRPM);
      SmartDashboard.putNumber("Shooter/Feeder/errorRPM", feederTargetRPM - feederActualRPM);
      SmartDashboard.putNumber("Shooter/Feeder/ff", feederFF);
      SmartDashboard.putNumber("Shooter/Feeder/pidOutput", feederPIDOutput);
      SmartDashboard.putNumber("Shooter/Feeder/totalVoltage", feederVoltage);
    }

    // --- HOPPER PID LOOP ---
    if (hopperPIDEnabled) {
      double hopperFF = (hopperTargetRPM / NEO_VORTEX_FREE_SPEED_RPM) * 12.0;
      double hopperPIDOutput = hopperPIDController.calculate(hopperActualRPM, hopperTargetRPM);
      double hopperVoltage = MathUtil.clamp(hopperFF + hopperPIDOutput, -12.0, 12.0);
      extraHopperRoller.setVoltage(hopperVoltage);

      SmartDashboard.putNumber("Shooter/Hopper/targetRPM", hopperTargetRPM);
      SmartDashboard.putNumber("Shooter/Hopper/errorRPM", hopperTargetRPM - hopperActualRPM);
      SmartDashboard.putNumber("Shooter/Hopper/ff", hopperFF);
      SmartDashboard.putNumber("Shooter/Hopper/pidOutput", hopperPIDOutput);
      SmartDashboard.putNumber("Shooter/Hopper/totalVoltage", hopperVoltage);
    }

    // Always display actual RPM for monitoring
    SmartDashboard.putNumber("Shooter/Launcher/actualRPM", launcherActualRPM);
    SmartDashboard.putNumber("Shooter/Feeder/actualRPM", feederActualRPM);
    SmartDashboard.putNumber("Shooter/Hopper/actualRPM", hopperActualRPM);
  }
}
