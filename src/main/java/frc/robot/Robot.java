// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static final String centerBlueThreeNote = "Blue center three note";
  private static final String BlueLeftOneNote = "Blue left shoot 'n go!";
  private static final String blueLeftTwoNote = "blue Left Two Note";
  private static final String rightSpeaker2NoteBlue = "Blue right side speaker! 2 note";

  private static final String centerRedThreeNote = "Red center three note";
  private static final String RedLeftOneNote = "Red left one note";
  private static final String RedRightOneNote = "Red right shoot 'n go!";
  private static final String redLeftTwoNoteNEW = "red (blue mimik) Left Two Note";
  private static final String redLeftTwoNote = "red Left Two Note";
  private static final String rightSpeaker2NoteRed = "Red right side speaker! 2 note";

  // private static final String rightSpeaker3Note = "right side speaker! 3 note";

  private static final String centerTwoNote = "center two note";
  private static final String goinStraight = "goin' straight";
  private double speed = 5.0;
  private double turnSpeed = 1.5;
  private double driveMode = 4;
  double Forward = 0;
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Variables
  XboxController controller1 = new XboxController(0);
  XboxController controller2 = new XboxController(1);
  CANSparkMax driveRightA = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax driveRightB = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax driveLeftA = new CANSparkMax(8, MotorType.kBrushless);
  CANSparkMax driveLeftB = new CANSparkMax(9, MotorType.kBrushless);
   double backward;
  Servo ampServo = new Servo(1);
  double turn = controller1.getRightX() / turnSpeed;
  TalonFX shooterA = new TalonFX(4);
  TalonFX shooterB = new TalonFX(3);
  VictorSPX hanger = new VictorSPX(19);
  CANSparkMax intake = new CANSparkMax(10, MotorType.kBrushless);
  public double autoStart = 0;
  public double wait = 0;
  public double rightPosition = -driveRightA.getEncoder().getPosition();
  public double leftPosition = driveLeftA.getEncoder().getPosition();
  double RPM;
  
  VelocityVoltage velocitySlow = new VelocityVoltage(RPM, 0, true, 0, 1, false, false, false);
  NeutralModeValue Coast = NeutralModeValue.Coast;
  TalonSRX amp = new TalonSRX(11);
  double kP = 0.05;
  String state = "init";
  double shootingSpeedA = 1500; //1500
  double shootingSpeedB = 1200; //1200
  VelocityVoltage velocity = new VelocityVoltage(shootingSpeedA, 0, true, 0, 0, false, false, false);
  private static final double flapDown = 1.0;
  private static final double flapUp = 0.5;

  ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  public Robot() {
    driveLeftB.follow(driveLeftA);
    driveRightB.follow(driveRightA);
  }

  // use driveForward(incert motor power here) to drive forward
  public void driveForward(double speed, double endTime) {
    drive(speed, speed, endTime);
  }

  // use tankDriveackward(incert motor power here) to drive backward
  public void tankDriveackward(double speed, double endTime) {
    drive(-speed, -speed, endTime);
  }

  // use turnRight(incert motor power here) to turn right
  public void turnLeft(double speed, double endTime) {
    drive(-speed, speed, endTime);

  }

  // use turnLeft(incert motor power here) to turn left
  public void turnRight(double speed, double endTime) {
    drive(speed, -speed, endTime);

  }

  // use drive(incert left speed here, incert right speed here) to drive (tank
  // drive)
  public void drive(double leftSpeed, double rightSpeed, double endTime) {
    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    if (endTime - autoTimeElapsed < 0.25) {
      leftSpeed = leftSpeed * (endTime - autoTimeElapsed) * 4;
      rightSpeed = rightSpeed * (endTime - autoTimeElapsed) * 4;
    }
    tankDrive(leftSpeed, rightSpeed);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    double divisor = 
      Math.max(
        1.0, 
        Math.max(
          Math.abs(leftSpeed), 
          Math.abs(rightSpeed)));
    driveLeftA.set(-leftSpeed / divisor);
    driveRightA.set(rightSpeed / divisor);

  }

  public double constrain(double number, double maximum, double minimum) {
    if (number > maximum) {
      return maximum;
    } else if (number < minimum) {
      return minimum;
    } else {
      return number;
    }
  }

  public void point(double direction, double maxSpeed) {
    // Find the heading error; setpoint is direction
    double error = direction - gyro.getAngle();

    // Turns the robot to face the desired direction
    tankDrive(constrain(kP * error, maxSpeed, -maxSpeed), constrain(-kP * error, maxSpeed, -maxSpeed));

  }

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture(0);
    m_chooser.addOption(blueLeftTwoNote, blueLeftTwoNote);
    m_chooser.addOption(BlueLeftOneNote, BlueLeftOneNote);
    m_chooser.addOption(rightSpeaker2NoteBlue, rightSpeaker2NoteBlue);
    m_chooser.addOption(centerBlueThreeNote, centerBlueThreeNote);
    m_chooser.addOption(redLeftTwoNoteNEW, redLeftTwoNoteNEW);
    m_chooser.addOption(rightSpeaker2NoteRed, rightSpeaker2NoteRed);
    m_chooser.addOption(RedRightOneNote, RedRightOneNote);
    // m_chooser.addOption(rightSpeaker3Note, rightSpeaker3Note);
    m_chooser.addOption(RedLeftOneNote, RedLeftOneNote);
    m_chooser.addOption(goinStraight, goinStraight);
    m_chooser.addOption(centerTwoNote, centerTwoNote);
    m_chooser.addOption(centerRedThreeNote, centerRedThreeNote);
    SmartDashboard.putData("Auto choices", m_chooser);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 0.6;
       configs.Slot0.kI = 0.55;
    //configs.Slot0.kD = 0.01;

    configs.Slot1.kP = 0.101;
    configs.Slot1.kI = 0.065;
    //configs.Slot1.kD = 0.01;

    configs.Voltage.PeakForwardVoltage = 11;
    configs.Voltage.PeakReverseVoltage = -11;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    configs.MotorOutput.NeutralMode = Coast;

    shooterA.getConfigurator().apply(configs);
    shooterB.getConfigurator().apply(configs);

    driveRightA.getEncoder().setPositionConversionFactor((6.0 * Math.PI / 12.0));
    driveLeftA.getEncoder().setPositionConversionFactor((6.0 * Math.PI / 12.0));

    gyro.reset();

    driveLeftA.restoreFactoryDefaults();
    driveRightA.restoreFactoryDefaults();
    driveLeftB.restoreFactoryDefaults();
    driveRightB.restoreFactoryDefaults();
    double voltageComp = 10.0;
    driveLeftA.enableVoltageCompensation(voltageComp);
    driveRightA.enableVoltageCompensation(voltageComp);
    driveLeftB.enableVoltageCompensation(voltageComp);
    driveRightB.enableVoltageCompensation(voltageComp);

    driveLeftA.setOpenLoopRampRate(0.5);
    driveRightA.setOpenLoopRampRate(0.5);

    driveLeftA.follow(ExternalFollower.kFollowerDisabled, 0);
    driveRightA.follow(ExternalFollower.kFollowerDisabled, 0);

    driveLeftB.follow(driveLeftA);
    driveRightB.follow(driveRightA);

  }

  @Override
  public void robotPeriodic() {
    rightPosition = -driveRightA.getEncoder().getPosition();
    leftPosition = driveLeftA.getEncoder().getPosition();

    SmartDashboard.putNumber("Right Position Var", rightPosition);
    SmartDashboard.putNumber("Left Position Var", leftPosition);

    SmartDashboard.putNumber("Overall Speed", (turn / speed) - Forward);

    SmartDashboard.putNumber("ShooterA", shootingSpeedA);
    SmartDashboard.putNumber("ShooterB", shootingSpeedB);

    SmartDashboard.putNumber("gyro angle", gyro.getAngle());
    SmartDashboard.putNumber("gyro rate", gyro.getRate());
    SmartDashboard.putString("state", state);

  }

  /*
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    double voltageComp = 8.0;
    driveLeftA.enableVoltageCompensation(voltageComp);
    driveRightA.enableVoltageCompensation(voltageComp);
    driveLeftB.enableVoltageCompensation(voltageComp);
    driveRightB.enableVoltageCompensation(voltageComp);
    stopEverything();
    driveRightA.getEncoder().setPosition(0);
    driveLeftA.getEncoder().setPosition(0);
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    autoStart = Timer.getFPGATimestamp();

    driveLeftB.follow(driveLeftA);
    driveRightB.follow(driveRightA);

    driveRightA.getEncoder().setPosition(0);
    driveLeftA.getEncoder().setPosition(0);

    gyro.reset();
    state = "init";

  }

  /* This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    rightPosition = -driveRightA.getEncoder().getPosition();
    leftPosition = driveLeftA.getEncoder().getPosition();

    System.out.println(m_autoSelected);
    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    SmartDashboard.putNumber("autoTimeElapsed", autoTimeElapsed);
    if (autoTimeElapsed >= 15) {
      stopEverything();
      return;
    }
    if (state == "fin") {
      stopEverything();
      return;
    }
    if (m_autoSelected == BlueLeftOneNote) {
      runBlueLeftOneNote(autoTimeElapsed);
    } else if (m_autoSelected == RedLeftOneNote) {
      runRedLeftOneNote(autoTimeElapsed);
    } else if (m_autoSelected == RedRightOneNote) {
      runRedRightOneNote(autoTimeElapsed);
    } else if (m_autoSelected == blueLeftTwoNote) {
      runblueLeftTwoNote(autoTimeElapsed);
    } else if (m_autoSelected == redLeftTwoNoteNEW) {
      runRedLeftTwoNoteNEW(autoTimeElapsed);
    } else if (m_autoSelected == redLeftTwoNote) {
      runRedLeftTwoNote(autoTimeElapsed);
    } else if (m_autoSelected.equals(rightSpeaker2NoteBlue)) {
      runRightSpeaker2NoteBlue(autoTimeElapsed);
    } else if (m_autoSelected.equals(rightSpeaker2NoteRed)) {
      runRightSpeaker2NoteRed(autoTimeElapsed);
    } else if (m_autoSelected.equals(goinStraight)) {
      runGoinStraight(autoTimeElapsed);
    } else if (m_autoSelected.equals(centerTwoNote)) {
      runCenterTwoNote(autoTimeElapsed);
    } else if (m_autoSelected.equals(centerBlueThreeNote)) {
      runBlueCenterThreeNote(autoTimeElapsed);
    } else if (m_autoSelected.equals(centerRedThreeNote)) {
      runRedCenterThreeNote(autoTimeElapsed);
    }
  }

  public void runBlueCenterThreeNote(double autoTimeElapsed) {
    if (state == "init") {
      System.out.println("Running init state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      if (autoTimeElapsed > 1) {
        state = "two";
      }
    }
    if (state == "two") {
      System.out.println("Running two state");

      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (autoTimeElapsed > 2) {
        state = "three";
      }
    }
    if (state == "three") {
      System.out.println("Running three state");
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      tankDrive(-0.5, -0.4);
      if (leftPosition > 35) {
        state = "four";
      }
    }
    if (state == "four") {
      System.out.println("Running three state");
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      tankDrive(0.5, 0.4);
      if (leftPosition < 8.75) {
        state = "five";
        wait = autoTimeElapsed + 1;
      }
    }
    if (state == "five") {
      System.out.println("Running init state");
      tankDrive(0, 0);
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      intake.set(0.125);
      if (wait < autoTimeElapsed) {
        state = "six";
        wait = autoTimeElapsed + 1;
      }
    }
    if (state == "six") {
      System.out.println("Running two state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (wait < autoTimeElapsed) {
        state = "seven";
        wait = autoTimeElapsed + 1;
      }
    }
    if (state == "seven") {
      System.out.println("Running two state");
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      
      intake.set(0);
      tankDrive(0.5, -0.5);
      if (leftPosition < 2.75) {
        state = "eight";
        tankDrive(0, 0);
      }
    }
    if (state == "eight") {
      System.out.println("Running two state");
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      intake.set(-0.5);
      tankDrive(-0.5, -0.4);
      if (leftPosition > 27.5) {
        state = "nine";
        tankDrive(0, 0);
      }
    }
    if (state == "nine") {
      System.out.println("Running nine state");
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      tankDrive(0.5, 0.4);
      if (leftPosition < 6) {
        state = "wait after nine";
        wait = autoTimeElapsed + 1;
      }
    }
    if (state == "wait after nine") {
      tankDrive(0, 0);
      if (autoTimeElapsed > wait) {
        state = "ten";
      }
    }
    if (state == "ten") {
      System.out.println("Running ten state");
      tankDrive(-0.5, 0.5);
      if (leftPosition > 2.25) {
        state = "eleven";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "eleven") {
      System.out.println("Running ten state");
      tankDrive(-0.5, 0.5);
      if (leftPosition > 2) {
        state = "twelve";
        wait = autoTimeElapsed + 0.25;
        tankDrive(0, 0);
      }
    }
    if (state == "eleven") {
      intake.set(0.125);
      if (wait < autoTimeElapsed) {
        state = "twelve";
        intake.set(0);
        shooterA.setControl(velocity.withVelocity(0));
        shooterB.setControl(velocity.withVelocity(0));
      }
    }
    if (state == "twelve") {
      intake.set(-0.5);
      if (wait < autoTimeElapsed) {
        state = "thirteen";
        intake.set(0);
        wait = autoTimeElapsed + 1;
        shooterA.setControl(velocity.withVelocity(0));
        shooterB.setControl(velocity.withVelocity(0));
      }
    }
    if (state == "thirteen") {
      System.out.println("Running init state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      if (autoTimeElapsed > wait) {
        state = "fourteen";
        wait = autoTimeElapsed + 0.5;
      }
    }
    if (state == "fourteen") {
      System.out.println("Running two state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (autoTimeElapsed > wait) {
        state = "fifteen";
      }
    }
  }

  public void runRedCenterThreeNote(double autoTimeElapsed) {
    if (state == "init") {
      System.out.println("Running init state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      if (autoTimeElapsed > 1) {
        state = "two";
      }
    }
    if (state == "two") {
      System.out.println("Running two state");

      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (autoTimeElapsed > 2) {
        state = "three";
      }
    }
    if (state == "three") {
      System.out.println("Running three state");
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      tankDrive(-0.5, -0.4);
      if (leftPosition > 35) {
        state = "four";
      }
    }
    if (state == "four") {
      System.out.println("Running three state");
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      tankDrive(0.5, 0.4);
      if (leftPosition < 8.75) {
        state = "five";
        wait = autoTimeElapsed + 1;
      }
    }
    if (state == "five") {
      System.out.println("Running init state");
      tankDrive(0, 0);
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      intake.set(0.125);
      if (wait < autoTimeElapsed) {
        state = "six";
        wait = autoTimeElapsed + 1;
      }
    }
    if (state == "six") {
      System.out.println("Running two state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (wait < autoTimeElapsed) {
        state = "seven";
        wait = autoTimeElapsed + 1;
      }
    }
    if (state == "seven") {
      System.out.println("Running two state");
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      intake.set(0);
      tankDrive(0.5, -0.5);
      if (leftPosition < 3.0) {
        state = "eight";
        tankDrive(0, 0);
      }
    }
    if (state == "eight") {
      System.out.println("Running two state");
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      intake.set(-0.5);
      tankDrive(-0.5, -0.4);
      if (leftPosition > 27.5) {
        state = "nine";
        tankDrive(0, 0);
      }
    }
    if (state == "nine") {
      System.out.println("Running nine state");
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      tankDrive(0.5, 0.4);
      if (leftPosition < 6) {
        state = "wait after nine";
        wait = autoTimeElapsed + 1;
      }
    }
    if (state == "wait after nine") {
      tankDrive(0, 0);
      if (autoTimeElapsed > wait) {
        state = "ten";
      }
    }
    if (state == "ten") {
      System.out.println("Running ten state");
      tankDrive(-0.5, 0.5);
      if (leftPosition > 3) {
        state = "eleven";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "eleven") {
      System.out.println("Running ten state");
      tankDrive(-0.5, 0.5);
      if (leftPosition > 2) {
        state = "twelve";
        wait = autoTimeElapsed + 0.25;
        tankDrive(0, 0);
      }
    }
    if (state == "eleven") {
      intake.set(0.125);
      if (wait < autoTimeElapsed) {
        state = "twelve";
        intake.set(0);
        shooterA.setControl(velocity.withVelocity(0));
        shooterB.setControl(velocity.withVelocity(0));
      }
    }
    if (state == "twelve") {
      intake.set(-0.5);
      if (wait < autoTimeElapsed) {
        state = "thirteen";
        intake.set(0);
        wait = autoTimeElapsed + 1;
        shooterA.setControl(velocity.withVelocity(0));
        shooterB.setControl(velocity.withVelocity(0));
      }
    }
    if (state == "thirteen") {
      System.out.println("Running init state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      intake.set(0.125);
      if (autoTimeElapsed > wait) {
        state = "fourteen";
        wait = autoTimeElapsed + 0.5;
      }
    }
    if (state == "fourteen") {
      System.out.println("Running two state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (autoTimeElapsed > wait) {
        state = "fifteen";
      }
    }
  }

  public void runCenterTwoNote(double autoTimeElapsed) {
    if (state == "init") {
      System.out.println("Running init state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      if (autoTimeElapsed > 1) {
        state = "two";
      }
    }
    if (state == "two") {
      System.out.println("Running two state");

      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (autoTimeElapsed > 2) {
        state = "three";
      }
    }
    if (state == "three") {
      System.out.println("Running three state");
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      tankDrive(-0.5, -0.4);
      if (leftPosition > 35) {
        state = "four";
      }
    }
    if (state == "four") {
      System.out.println("Running three state");
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      tankDrive(0.5, 0.4);
      if (leftPosition > 2.5) {
        state = "five";
        wait = autoTimeElapsed + 1;
      }
    }
    if (state == "five") {
      System.out.println("Running init state");
      tankDrive(0, 0);
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      if (wait < autoTimeElapsed) {
        state = "six";
        wait = autoTimeElapsed + 1;
      }
    }
    if (state == "six") {
      System.out.println("Running two state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (wait < autoTimeElapsed) {
        state = "fin";
      }
    }
  }

  /**
   * stops all motors
   */
  public void stopEverything() {
    tankDrive(0, 0);
    shooterA.setControl(velocity.withVelocity(0));
    shooterB.setControl(velocity.withVelocity(0));
    intake.set(0);
  }

  /**
   * One note, in front of speaker
   */
  public void runBlueLeftOneNote(double autoTimeElapsed) {
    System.out.println("Entering oneNoteAuto block");
    if (state == "init") {
      System.out.println("Running init state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      if (autoTimeElapsed > 1) {
        state = "two";
      }
    }
    if (state == "two") {
      System.out.println("Running two state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (autoTimeElapsed > 2) {
        state = "three";
      }
    }
    if (state == "three") {
      tankDrive(-0.5, -0.625);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      intake.set(0);
      if (rightPosition > 50) {
        state = "fin";
      }
    }

  }

  public void runRedRightOneNote(double autoTimeElapsed) {
    System.out.println("Entering oneNoteAuto block");
    if (state == "init") {
      System.out.println("Running init state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      if (autoTimeElapsed > 1) {
        state = "two";
      }
    }
    if (state == "two") {
      System.out.println("Running two state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (autoTimeElapsed > 2) {
        state = "three";
      }
    }
    if (state == "three") {
      tankDrive(-0.6, -0.625);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      intake.set(0);
      if (rightPosition > 50) {
        state = "fin";
      }
    }

  }

  /**
   * One note, in front of speaker
   */
  public void runRedLeftOneNote(double autoTimeElapsed) { // NOT A SHOOT N' GO
    System.out.println("Entering oneNoteAuto block");
    if (state == "init") {
      System.out.println("Running init state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      if (autoTimeElapsed > 1) {
        state = "two";
      }
    }
    if (state == "two") {
      System.out.println("Running two state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (autoTimeElapsed > 2) {
        state = "three";
      }
    }
    if (state == "three") {
      tankDrive(0.4, -0.75);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      intake.set(0);
      if (rightPosition > 2.6) {
        state = "four";
        wait = autoTimeElapsed + 10;
      }
    }
    if (state == "four") {
      tankDrive(0, 0);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      intake.set(0);
      if (autoTimeElapsed > wait) {
        state = "five";
      }
    }
    if (state == "five") {
      tankDrive(-0.5, -0.5);
      if (rightPosition > 40) {
        state = "six";
      }
    }
    if (state == "six") {
      intake.set(-0.5);
      tankDrive(-0.5, -0.5);
      if (rightPosition > 50) {
        state = "fin";
      }
    }
  }

  /**
   * 2 note, starts on right side, shoots preloaded, picks up and shoots the note
   * closest to the amp
   */
  public void runRightSpeaker2NoteBlue(double autoTimeElapsed) {
    if (state == "init") {
      System.out.println("Running init state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      if (autoTimeElapsed > 1) {
        state = "two";
      }
    }
    if (state == "two") {
      System.out.println("Running two state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (autoTimeElapsed > 2) {
        state = "three";
      }
    }
    if (state == "three") {
      tankDrive(-0.5, -0.5);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      if (leftPosition > 1 && rightPosition < 1) {
        state = "four";
        tankDrive(0, 0);
      }
    }
    if (state == "four") {
      tankDrive(-0.5, 0.5);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      if (leftPosition > 10.25) {
        state = "five";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "five") {
      tankDrive(-0.5, -0.5);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      intake.set(-0.5);
      if (leftPosition > 45) {
        state = "six";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "six") {
      tankDrive(0, 0);
      intake.set(-0.75);
      if (wait < autoTimeElapsed) {
        state = "seven";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "seven") {
      tankDrive(0.4, 0.4);
      intake.set(0);
      if (leftPosition < 15) {
        state = "eight";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "eight") {
      tankDrive(0.5, -0.5);
      intake.set(0);
      if (leftPosition < 4) {
        state = "nine";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "nine") {
      intake.set(0.25);
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      if (wait < autoTimeElapsed) {
        state = "ten";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "ten") {
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (wait < autoTimeElapsed) {
        state = "fin"; // INITAIL FIN ADD LATER
      }
    }
  }

  /**
   * 2 note, starts on right side, shoots preloaded, picks up and shoots the note
   * closest to the amp
   */
  public void runRightSpeaker2NoteRed(double autoTimeElapsed) {
    if (state == "init") {
      System.out.println("Running init state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      if (autoTimeElapsed > 1) {
        state = "two";
      }
    }
    if (state == "two") {
      System.out.println("Running two state");

      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (autoTimeElapsed > 2) {
        state = "three";
      }
    }
    if (state == "three") {
      tankDrive(-0.5, -0.5);
      intake.set(0);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      if (leftPosition > 1 && rightPosition < 1) {
        state = "four";
        tankDrive(0, 0);
      }
    }
    if (state == "four") {
      tankDrive(-0.5, 0.5);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      if (leftPosition > 8.0) {
        state = "five";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "five") {
      tankDrive(-0.5, -0.5);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      intake.set(-0.5);
      if (leftPosition > 37) {
        state = "six";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "six") {
      tankDrive(0, 0);
      intake.set(-0.75);
      if (wait < autoTimeElapsed) {
        state = "seven";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "seven") {
      tankDrive(0.4, 0.4);
      intake.set(0);
      if (leftPosition < 20) {
        state = "eight";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "eight") {
      tankDrive(0.5, -0.5);
      intake.set(0);
      if (leftPosition < 6) {
        state = "nine";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "nine") {
      intake.set(0.25);
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      if (wait < autoTimeElapsed) {
        state = "ten";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "ten") {
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (wait < autoTimeElapsed) {
        state = "fin"; // INITAIL FIN ADD LATER
      }
    }
  }

  /**
   * 3 note, starts on right side, shoots preloaded, picks up and shoots the note
   * closest to the amp, then picks up and shoots far note closest to the wall
   */
  public void runRightSpeaker3Note(double autoTimeElapsed) {
    System.out.println("entering triple note slanted block");
    if (state == "init") {
      System.out.println("Running init state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      if (autoTimeElapsed > 1) {
        state = "two";
      }
    }
    if (state == "two") {
      System.out.println("Running two state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      if (autoTimeElapsed > 2) {
        state = "three";
      }
    }
    if (state == "three") {
      tankDrive(-0.5, -0.5);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      if (leftPosition > 1 && rightPosition < 1) {
        state = "four";
        tankDrive(0, 0);
      }
    }
    if (state == "four") {
      tankDrive(-0.5, 0.5);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      if (leftPosition > 8.25) {
        state = "five";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "five") {
      tankDrive(-0.5, -0.5);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      intake.set(-0.5);
      if (leftPosition > 42) {
        state = "six";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "six") {
      tankDrive(0, 0);
      intake.set(-0.75);
      if (wait < autoTimeElapsed) {
        state = "seven";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "seven") {
      tankDrive(0.4, 0.4);
      intake.set(0);
      if (leftPosition < 15) {
        state = "eight";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "eight") {
      tankDrive(0.5, -0.5);
      intake.set(0);
      if (leftPosition < 6) {
        state = "nine";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "nine") {
      intake.set(0.25);
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      if (wait < autoTimeElapsed) {
        state = "ten";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "ten") {
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (wait < autoTimeElapsed) {
        state = "eleven"; // INITAIL FIN ADD LATER
      }
    }
    if (state == "eleven") {
      tankDrive(-0.5, 0.5);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      if (leftPosition > 5) { // 8.75 = mid note
        state = "twelve";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "twelve") {
      tankDrive(-1, -0.75);
      if (leftPosition > 130) {
        state = "thirteen";
        tankDrive(0, 0);
      }
    }
  }

  /* This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    double voltageComp = 10.0;
    driveLeftA.enableVoltageCompensation(voltageComp);
    driveRightA.enableVoltageCompensation(voltageComp);
    driveLeftB.enableVoltageCompensation(voltageComp);
    driveRightB.enableVoltageCompensation(voltageComp);

    driveLeftB.follow(driveLeftA);
    driveRightB.follow(driveRightA);

    driveRightA.getEncoder().setPosition(0);
    driveLeftA.getEncoder().setPosition(0);
    gyro.reset();
  }

  /* This function is called periodically during operator control. */
  @Override

  public void teleopPeriodic() {
    SmartDashboard.putNumber("shootingSpeeda",shooterA.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("shootingSpeedb",shooterB.getVelocity().getValueAsDouble());
    // SPEED CONTROLS
    if (controller1.getLeftBumper()) {
      speed = 2.5;
      turnSpeed = 3;
      
    } else if (controller1.getRightBumper()) {
      speed = 0.5;
      turnSpeed = 0.5;
      
    }
    //  else if (controller1.getRightTriggerAxis() > 0.25) {
    //   turnSpeed = 7.5;
      
    // }
     else {
      speed = 0.85;
      turnSpeed = 0.5;
    }
    turn = (-controller1.getRightX() * -turnSpeed);
    Forward = controller1.getRightTriggerAxis();
    backward = controller1.getLeftTriggerAxis();
    tankDrive(((-Forward + backward * speed) - (turn)) * 1.04, ((-Forward + backward * speed) + (turn)) * 0.8);
    // DROVE DATA)
    SmartDashboard.putNumber(" drive speed", ((turn / speed) - Forward));
    SmartDashboard.putNumber(" turning speed", turn);
    SmartDashboard.putNumber(" speed", speed);
    SmartDashboard.putNumber(" drive mode", driveMode);

    // JOYSTICK CONTROLS + TRIGGER
    SmartDashboard.putNumber("left speed", ((turn / speed) - Forward));
    SmartDashboard.putNumber("right speed", (turn / speed) + Forward);
    // TRIGGER CONTROLS (no turn yet)

    // MANIPULATOR CONTROLS
    // SHOOTER SPEED CONTROLS
    if (controller2.getXButton()) { // slow
      shootingSpeedA = 100; //100 acts as a "sweetspot"!!!
    } else if (controller2.getYButton()) { // fast
      shootingSpeedA = 2000;
    } else { // normal
      shootingSpeedA = 5500;//1500
    }
    shootingSpeedB = shootingSpeedA * 0.8;
    // SHOOT
    if (controller2.getRightTriggerAxis() > 0.5) {
      shooterA.setControl(velocity.withVelocity(shootingSpeedA/60));
      shooterB.setControl(velocity.withVelocity(shootingSpeedB/60));
    }
    // INTAKE
    else if (controller2.getRightBumper()) {
      shooterB.setControl(velocitySlow.withVelocity(-10));
      shooterA.setControl(velocitySlow.withVelocity(-10));
    } else {
      shooterA.set(0);
      shooterB.set(0);
    }
    // INTAKE
    if (controller2.getAButton()) {
      intake.set(-0.5);
    }
    // OUTTAKE
    else if (controller2.getBButton()) {
      intake.set(0.5);
    } else {
      intake.set(0);
    }

    // AMP
    // INTAKE
    if (controller2.getLeftBumper()) {
      ampServo.set(flapDown);
    }
    // OUTTAKE
    else if (controller2.getLeftTriggerAxis() > 0.5) {
      ampServo.set(flapUp);
      shooterA.setControl(velocitySlow.withVelocity(30));
      shooterB.setControl(velocitySlow.withVelocity(30));
    }
    // HANGING
    if (controller2.getLeftY() >= 0.2) {
      hanger.set(ControlMode.PercentOutput, -0.75);
    } else if (controller2.getLeftY() <= -0.2) {
      hanger.set(ControlMode.PercentOutput, 0.75);
    } else {
      hanger.set(ControlMode.PercentOutput, 0);
    }

  }

  /* This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /* This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /* This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /* This function is called periodically during test mode. */
  @Override
  public void testPeriodic() { // what is this for?
  }

  /* This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /* This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  /**
   * unused auto
   */
  public void runGoinStraight(double autoTimeElapsed) {
    if (state == "init") {
      if (autoTimeElapsed > 10) {
        state = "two";
      }
    }
    if (state == "two") {
      tankDrive(-0.5, -0.4);
      if (leftPosition > 50) {
        state = "fin";
      }
    }
  }

  public void runblueLeftTwoNote(double autoTimeElapsed) {
    System.out.println("entering two note slanted block");
    if (state == "init") {
      System.out.println("Running init state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      if (autoTimeElapsed > 0.5) {
        state = "two";
      }
    }
    if (state == "two") {
      System.out.println("Running two state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (autoTimeElapsed > 1) {
        state = "three";
      }
    }
    if (state == "three") {
      tankDrive(-0.5, -0.5);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      if (driveLeftA.getEncoder().getPosition() > 0.125 && driveRightA.getEncoder().getPosition() < -0.125) {
        state = "four";
        tankDrive(0, 0);
      }
    }
    if (state == "four") {
      tankDrive(0.5, -0.5);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      if (driveRightA.getEncoder().getPosition() < -4.0) {
        state = "five";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "five") {
      tankDrive(-0.5, -0.4);
      intake.set(-0.5);
      if (driveRightA.getEncoder().getPosition() < -35) {
        state = "six";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "six") {
      tankDrive(0.5, 0.4);
      intake.set(-0.5);
      if (driveRightA.getEncoder().getPosition() > -20) {
        state = "seven";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "seven") {
      tankDrive(-0.5, 0.5);
      intake.set(-0.5);
      if (driveRightA.getEncoder().getPosition() > -11) {
        state = "eight";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "eight") { // SHOOT SECOND NOTE? nah, part one.
      intake.set(.125);
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      if (autoTimeElapsed > wait) {
        state = "nine";
        intake.set(0);
        tankDrive(0, 0);
        wait = autoTimeElapsed + 1;
      }
    }
    if (state == "nine") { // SHOOT SECOND NOTE!!! FOR REAL THIS TIME!!!!! :)
      intake.set(-.75);
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (autoTimeElapsed > wait) {
        state = "fin";
      }
    }
  }

  public void runRedLeftTwoNoteNEW(double autoTimeElapsed) {
    System.out.println("entering two note slanted block");
    if (state == "init") {
      System.out.println("Running init state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      if (autoTimeElapsed > 0.5) {
        state = "two";
      }
    }
    if (state == "two") {
      System.out.println("Running two state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (autoTimeElapsed > 1) {
        state = "three";
      }
    }
    if (state == "three") {
      tankDrive(-0.5, -0.5);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      if (driveLeftA.getEncoder().getPosition() > 0.125 && driveRightA.getEncoder().getPosition() < -0.125) {
        state = "four";
        tankDrive(0, 0);
      }
    }
    if (state == "four") {
      tankDrive(0.5, -0.5);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      if (driveRightA.getEncoder().getPosition() < -5.0) {
        state = "five";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "five") {
      tankDrive(-0.5, -0.4);
      intake.set(-0.5);
      if (driveRightA.getEncoder().getPosition() < -35) {
        state = "six";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "six") {
      tankDrive(0.5, 0.4);
      intake.set(-0.5);
      if (driveRightA.getEncoder().getPosition() > -20) {
        state = "seven";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "seven") {
      tankDrive(-0.5, 0.5);
      intake.set(-0.5);
      if (driveRightA.getEncoder().getPosition() > -9) {
        state = "eight";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "eight") { // SHOOT SECOND NOTE? nah, part one.
      intake.set(.125);
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      if (autoTimeElapsed > wait) {
        state = "nine";
        intake.set(0);
        tankDrive(0, 0);
        wait = autoTimeElapsed + 1;
      }
    }
    if (state == "nine") { // SHOOT SECOND NOTE!!! FOR REAL THIS TIME!!!!! :)
      intake.set(-.75);
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (autoTimeElapsed > wait) {
        state = "fin";
      }
    }
  }

  public void runRedLeftTwoNote(double autoTimeElapsed) {
    System.out.println("entering two note slanted block");
    if (state == "init") {
      System.out.println("Running init state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      if (autoTimeElapsed > 1) {
        state = "two";
      }
    }
    if (state == "two") {
      System.out.println("Running two state");
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      if (autoTimeElapsed > 2) {
        state = "three";
      }
    }
    if (state == "three") {
      tankDrive(-0.5, -0.5);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      if (driveLeftA.getEncoder().getPosition() > 1 && driveRightA.getEncoder().getPosition() < -1) {
        state = "four";
        tankDrive(0, 0);
      }
    }
    if (state == "four") {
      tankDrive(0.5, -0.5);
      shooterA.setControl(velocity.withVelocity(0));
      shooterB.setControl(velocity.withVelocity(0));
      if (driveRightA.getEncoder().getPosition() < -10.25) {
        state = "five";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }

    if (state == "five") {
      tankDrive(0, 0);
      if (wait < autoTimeElapsed) {
        state = "six";
        tankDrive(0, 0);
      }
    }
    if (state == "six") {
      tankDrive(-0.25, -0.25);
      intake.set(-0.5);
      if (driveRightA.getEncoder().getPosition() < -35) {
        state = "seven";
        wait = autoTimeElapsed + 1;
        tankDrive(0, 0);
      }
    }
    if (state == "seven") { // BEGIN TO GO IN REVERSE
      tankDrive(0, 0);
      intake.set(-0.75);
      if (autoTimeElapsed > wait) {
        state = "eight";
        tankDrive(0, 0);
      }
    }
    if (state == "eight") {
      tankDrive(0.25, 0.25);
      intake.set(0);
      if (driveRightA.getEncoder().getPosition() > -11.25) {
        state = "nine";
        intake.set(0);
        tankDrive(0, 0);
      }
    }
    if (state == "nine") { // SHOOT SECOND NOTE!!! YEAH THE HOME STRETCH, wait no turn
      tankDrive(-0.5, 0.5);
      if (driveRightA.getEncoder().getPosition() > -4.0) {
        state = "ten";
        intake.set(0);
        tankDrive(0, 0);
        wait = autoTimeElapsed + 1;
      }
    }
    if (state == "ten") { // SHOOT SECOND NOTE? nah, part one.
      intake.set(.25);
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(0));
      if (autoTimeElapsed > wait) {
        state = "eleven";
        intake.set(0);
        tankDrive(0, 0);
        wait = autoTimeElapsed + 1;
      }
    }
    if (state == "eleven") { // SHOOT SECOND NOTE!!! FOR REAL THIS TIME!!!!! :)
      intake.set(-.75);
      shooterA.setControl(velocity.withVelocity(1500));
      shooterB.setControl(velocity.withVelocity(1200));
      intake.set(-0.5);
      if (autoTimeElapsed > wait) {
        state = "fin";
      }
    }
  }

}