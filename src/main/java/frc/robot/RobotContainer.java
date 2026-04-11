// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.TunableNumber;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveParser;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer
{
  private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                           "swerve"));
  // Shares the same physical motors (CAN 51 & 52) as the shooter
  private final CANFuelSubsystem m_fuel = new CANFuelSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();

  private final CommandXboxController m_driverXbox =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_secondaryDriverXbox =
      new CommandXboxController(OperatorConstants.kSecondaryDriverControllerPort);

  TunableNumber m_angleP = new TunableNumber("Swerve/PID/ModuleAngle/P", SwerveParser.pidfPropertiesJson.angle.p);
  TunableNumber m_angleD = new TunableNumber("Swerve/PID/ModuleAngle/D", SwerveParser.pidfPropertiesJson.angle.d);
  TunableNumber m_driveP = new TunableNumber("Swerve/PID/ModuleDrive/P", SwerveParser.pidfPropertiesJson.drive.p);
  TunableNumber m_driveD = new TunableNumber("Swerve/PID/ModuleDrive/D", SwerveParser.pidfPropertiesJson.drive.d);

  private SendableChooser<Command> m_autoChooser = null;

  public RobotContainer() {
    // --- Limelight camera stream for Elastic/Shuffleboard/SmartDashboard ---
    // Port 5800 = raw MJPEG stream of what the Limelight camera sees
    // If "limelight.local" doesn't resolve, use the static IP (typically 10.76.68.11)
    HttpCamera limelightCam = new HttpCamera("limelight", "http://10.76.68.11:5800/stream.mjpg");
    CameraServer.startAutomaticCapture(limelightCam);

    // Field-oriented drive (default)
    NamedCommands.registerCommand("Intake", m_fuel.intake().withTimeout(3));
    NamedCommands.registerCommand("adjustShoot", m_fuel.adjustingShoot(m_vision));
    NamedCommands.registerCommand("auto_fixed_voltage_shoot", m_fuel.auto_shoot().withTimeout(7));
    NamedCommands.registerCommand("maxShoot", m_fuel.maxShoot().withTimeout(7));
    Command driveFieldOrientedAnglularVelocity = m_drivebase.driveFieldOrientedCommand(
        () -> MathUtil.applyDeadband(m_driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverXbox.getHID().getRawAxis(2) * -1, OperatorConstants.RIGHT_X_DEADBAND),
        () -> false
    );

    // Robot-oriented drive (left bumper)
    Command driveRobotOriented = m_drivebase.driveRobotRelativeCommand(
        () -> MathUtil.applyDeadband(m_driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverXbox.getHID().getRawAxis(2) * -1, OperatorConstants.RIGHT_X_DEADBAND),
        () -> false
    );
    addCommandToDashboard(driveRobotOriented);

    
    Command zeroGyro = m_drivebase.runOnce(() -> m_drivebase.zeroGyro()).withName("zeroGyro");
    addCommandToDashboard(zeroGyro);

    Command resetOdometrytoAllianceZero = m_drivebase.runOnce(
        () -> m_drivebase.resetOdometry(m_drivebase.invertIfFieldFlipped(new Pose2d(0, 0, new Rotation2d()))))
        .withName("resetOdometrytoAllianceZero");
    addCommandToDashboard(resetOdometrytoAllianceZero);

    Command addFakeVisionReading = m_drivebase.runOnce(() -> m_drivebase.addFakeVisionReading())
        .withName("addFakeVisionReading");
    addCommandToDashboard(addFakeVisionReading);

    Command testMotors = m_drivebase.run(() -> {
      SwerveDriveTest.powerAngleMotorsDutyCycle(m_drivebase.swerveDrive, m_driverXbox.getLeftX());
      SwerveDriveTest.powerDriveMotorsDutyCycle(m_drivebase.swerveDrive, m_driverXbox.getLeftY());
    }).withName("testMotors");
    addCommandToDashboard(testMotors);

    Command testAngleMotors = m_drivebase.run(() -> {
      SwerveDriveTest.angleModules(m_drivebase.swerveDrive, 
        // Divided by 2 because want to go -180 to 180, not full circle
        Rotation2d.fromRotations(m_driverXbox.getLeftX() / 2));
    }).withName("testAngleMotors");
    addCommandToDashboard(testAngleMotors);

    TunableNumber angle = new TunableNumber("testAngle", 90);
    Command testSetAngle = m_drivebase.runEnd(
        () -> SwerveDriveTest.angleModules(m_drivebase.swerveDrive, Rotation2d.fromDegrees(angle.get())),
        () -> SwerveDriveTest.angleModules(m_drivebase.swerveDrive, Rotation2d.fromDegrees(0)))
        .withName("testSetAngle");
    addCommandToDashboard(testSetAngle);

    /*
     * Command testDriveToPose = drivebase.runOnce(
     * () -> drivebase.resetOdometry(new Pose2d(0, 0, new Rotation2d()))).andThen(
     * drivebase.driveToPose(
     * new Pose2d(new Translation2d(1, 1), Rotation2d.fromDegrees(0))))
     * .withName("testDriveToPose");
     */
    Command testDriveToPose = m_drivebase.testDriveToPose(
      new Pose2d(new Translation2d(Meters.of(0), Meters.of(0.1)),
                                   new Rotation2d(Degrees.of(0))))
        /* .asProxy() */.withName("testDriveToPose");
    addCommandToDashboard(testDriveToPose);

    m_drivebase.setDefaultCommand(
        // testMotors);
    driveRobotOriented);
        // driveRobotOriented);
        // driveFieldOrientedDirectAngle);
        // !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle :
        // driveFieldOrientedDirectAngleSim);
        //driveFieldOrient
        //edAnglularVelocity);
  
    // X = Intake
    m_secondaryDriverXbox.a().whileTrue(m_fuel.intake());
  
    // LT (Hold First) - launcher
    m_secondaryDriverXbox.leftBumper().whileTrue(m_fuel.shoot_launcher());

    // RT (hold later) - feeder
    m_secondaryDriverXbox.rightBumper().whileTrue(m_fuel.shoot_feeder());


    // B = Reverse Intake (hold)
    m_secondaryDriverXbox.x().whileTrue(m_fuel.reverseIntake());

    // A = Ferry (lower-power launch)
    m_secondaryDriverXbox.b().whileTrue(m_fuel.ferry());

    m_secondaryDriverXbox.y().whileTrue(m_fuel.maxShoot());

    // D-pad Up = Adjusting shoot (voltage based on Limelight distance)
    m_secondaryDriverXbox.povUp().whileTrue(m_fuel.adjustingShoot(m_vision));

    // D-pad Down = Test shooter RPM (manual RPM from SmartDashboard, for PID tuning)
    m_secondaryDriverXbox.povDown().whileTrue(m_fuel.testShooterRPM(m_vision));

    // D-pad Right = Simple adjusting shoot (ta → RPM formula, no lookup table needed)
    m_secondaryDriverXbox.povRight().whileTrue(m_fuel.adjustingShootSimple(m_vision));

    // D-pad Left = PID test (all 3 motors at shooting RPM using closed-loop PID)
    m_secondaryDriverXbox.povLeft().whileTrue(m_fuel.testPID());

   
////////// CHECK IF WE WANT THIS
    
    
    //m_fuel.setDefaultCommand(m_fuel.intake());

    /*
    Command disableArm = m_arm.runOnce(
    addCommandToDashboard(disableArm);
*/

    SmartDashboard.putData(CommandScheduler.getInstance());

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Add SysId drive characterization command to SmartDashboard for manual start
    SmartDashboard.putData("SysId/Drive Characterization", m_drivebase.driveCharacterizationSysIdCommand());
  }

  private void addCommandToDashboard(Command cmd) {
    SmartDashboard.putData("cmd/" + cmd.getName(), cmd);
  }

  public void simulationInit() {
   
  }

  public void robotPeriodic() {


    SmartDashboard.putNumber("joystick/left-X", m_driverXbox.getLeftX());
    SmartDashboard.putNumber("joystick/left-Y", m_driverXbox.getLeftY());
    SmartDashboard.putNumber("joystick/right-X", m_driverXbox.getRightX());
    SmartDashboard.putNumber("joystick/right-Y", m_driverXbox.getRightY());

    // Debug: show ALL raw axes to find which axis the Logitech maps right stick X to
    for (int i = 0; i < 6; i++) {
      SmartDashboard.putNumber("joystick/raw-axis-" + i, m_driverXbox.getHID().getRawAxis(i));
    }

    SmartDashboard.putNumber("pose/x", m_drivebase.getPose().getX());
    SmartDashboard.putNumber("pose/y", m_drivebase.getPose().getY());
    SmartDashboard.putNumber("pose/z", m_drivebase.getPose().getRotation().getDegrees());

    SmartDashboard.putString("alliance", m_drivebase.isFieldFlipped() ? "RED" : "BLUE");

    // Display SysId characterization constants
    SmartDashboard.putNumber("SysId/kS", frc.robot.Constants.SwerveConstants.kDriveS.get());
    SmartDashboard.putNumber("SysId/kV", frc.robot.Constants.SwerveConstants.kDriveV.get());
    SmartDashboard.putNumber("SysId/kA", frc.robot.Constants.SwerveConstants.kDriveA.get());


    // LIMELIGHT

    //SmartDashboard.putData("Limelight vision", frc.robot.subsystems.VisionSubsystem)

    if (m_angleD.hasChanged() || m_angleP.hasChanged()) {
      for (SwerveModule module : m_drivebase.swerveDrive.getModules()) {
        module.getAngleMotor().configurePIDF(new PIDFConfig(m_angleP.get(), m_angleD.get()));
      }
    }
    if (m_driveD.hasChanged() || m_driveP.hasChanged()) {
      for (SwerveModule module : m_drivebase.swerveDrive.getModules()) {
        module.getDriveMotor().configurePIDF(new PIDFConfig(m_driveP.get(), m_driveD.get()));
      }
    }
    // Update feedforward when tunable values are changed on SmartDashboard
    if (frc.robot.Constants.SwerveConstants.kDriveS.hasChanged()
        || frc.robot.Constants.SwerveConstants.kDriveV.hasChanged()
        || frc.robot.Constants.SwerveConstants.kDriveA.hasChanged()) {
      m_drivebase.swerveDrive.replaceSwerveModuleFeedforward(
          new SimpleMotorFeedforward(
              frc.robot.Constants.SwerveConstants.kDriveS.get(),
              frc.robot.Constants.SwerveConstants.kDriveV.get(),
              frc.robot.Constants.SwerveConstants.kDriveA.get()));
    }

    // Update shooter PID when tunable values are changed on SmartDashboard
    if (frc.robot.Constants.FuelConstants.kShooterP.hasChanged()
        || frc.robot.Constants.FuelConstants.kShooterI.hasChanged()
        || frc.robot.Constants.FuelConstants.kShooterD.hasChanged()
        || frc.robot.Constants.FuelConstants.kHopperP.hasChanged()
        || frc.robot.Constants.FuelConstants.kHopperI.hasChanged()
        || frc.robot.Constants.FuelConstants.kHopperD.hasChanged()) {
      m_fuel.reconfigurePID();
    }
  }

  public Command getAutonomousCommand() {
    // return new PathPlannerAuto("test auto");
     return m_autoChooser.getSelected();
    // return drivebase.getAutonomousCommand("small path");

    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
        
    /*return new SequentialCommandGroup(
      m_drivebase.driveAtSpeed(5, 0, 0, false).withTimeout(1.2)
     // drivebase.driveAtSpeed(-5, 0, 0, false).withTimeout(0.5) );*/
 }

  public void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }
}