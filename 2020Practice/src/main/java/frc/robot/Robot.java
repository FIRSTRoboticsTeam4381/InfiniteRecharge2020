/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.hal.sim.DriverStationSim;
import edu.wpi.first.hal.sim.mockdata.DriverStationDataJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static final String kDefaultAuto = "Default";
  private static final String KPortalAuto = "In Front of Portal";
  private static final String kDriveShoot = "Drive Forward and Shoot";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public CANSparkMax sequencer;
  // public SparkMax HandleExtension;
  public CANSparkMax shootTop;
  public CANSparkMax shootBottom;
  public CANEncoder shootTopEnc;
  public CANEncoder shootBottomEnc;
  public CANEncoder sequenceEnc;
  // public SparkMax HandleDrive;
  public CANPIDController indexPID;

  public WPI_VictorSPX kicker;
  public WPI_VictorSPX Intake;
  public WPI_VictorSPX r2;
  public WPI_VictorSPX l2;

  public WPI_TalonSRX climb;

  public WPI_TalonSRX ControlPanel;
  public WPI_TalonSRX r1;
  public WPI_TalonSRX l1;
  // public WPI_TalonSRX PickupArm;
  //
  public WPI_TalonSRX shootTurret;

  public WPI_TalonSRX spool;

  // Vision vars
  public double distoff = 0;
  public double size = 0;
  public double midx1 = 0;
  public double midx2 = 0;
  private double tensortime = 0;

  public double offsetTar = -10; // Change Offsett value
  public double offsetBall = 20; // Change Offsett value

  public double sizeCheck = 70;

  public double searchspeed = -.15;
  public double searchspeedTel = .15;

  public int t = 0;
  public boolean i = false;
  public boolean gottar = false;

  public String Visionclass;

  public double Turnvalball = 0;
  public double Speedvalball = 0;

  private double camnum = 0;

  public double Turnvaltar = 0;

  double time;

  // Joysticks
  private Joystick drStick = new Joystick(0);
  private Joystick spStick = new Joystick(1);
  private Joystick climbStick = new Joystick(2);
  private DifferentialDrive drive;

  public int lStop = 50; // CHANGE LATER
  public int rStop = -31200; // CHANGE LATER

  private double kP = 0.00009;
  private double kI = 0;
  private double kD = 0.000003;
  private double kIz = 0;
  private double kFF = 0.0003;
  private double kP1 = 0.00009;
  private double kI1 = 0;
  private double kD1 = 0.000003;
  private double kIz1 = 0;
  private double kFF1 = 0.0003;
  private double kMaxOutput = 1;
  private double kMinOutput = -1;

  private double sP = 0.1;
  private double sI = 1e-4;
  private double sD = 1;
  private double sIz = 0;
  private double sFF = 0;
  private double sMaxOutput = 1;
  private double sMinOutput = -1;

  public CANPIDController topWheelPID;
  public CANPIDController botWheelPID;

  public boolean autoEnd = false;
  public int stage = 1;
  public int tempLeft;
  public int tempRight;
  public double tempAutoShoot;
  public double indexIncrement = 8.5;
  public boolean shoot = false;
  public double tempShoot = 0;
  public AHRS ahrs;
  public boolean increment = false;

  DigitalInput in;
  //DigitalInput out;

  int ballcount = 0;
  boolean gotball = false;

  /**
   * 1 This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    Functions.robot = this;

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Full Portal ", KPortalAuto);
    m_chooser.addOption("Drive Forward and Shoot", kDriveShoot);

    SmartDashboard.putData("Auto choices", m_chooser);

    ahrs = new AHRS(SPI.Port.kMXP);

    r1 = new WPI_TalonSRX(1);
    r2 = new WPI_VictorSPX(2);
    l1 = new WPI_TalonSRX(3);
    l2 = new WPI_VictorSPX(4);
    /*
     * r1 = new WPI_TalonSRX(6); r2 = new WPI_VictorSPX(3); l1 = new
     * WPI_TalonSRX(4); l2 = new WPI_VictorSPX(1);
     */
    r2.follow(r1);
    l2.follow(l1);

    // shootTurret = new WPI_TalonSRX(9);
    shootTurret = new WPI_TalonSRX(5);
    shootTurret.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    shootTurret.setNeutralMode(NeutralMode.Brake);
    // shootTop = new CANSparkMax(45, MotorType.kBrushless);
    // shootBottom = new CANSparkMax(53, MotorType.kBrushless);
    shootTop = new CANSparkMax(50, MotorType.kBrushless);
    shootBottom = new CANSparkMax(56, MotorType.kBrushless);

    shootTopEnc = new CANEncoder(shootTop);
    shootBottomEnc = new CANEncoder(shootBottom);

    shootBottom.restoreFactoryDefaults();
    shootTop.restoreFactoryDefaults();

    botWheelPID = shootBottom.getPIDController();
    botWheelPID.setP(kP1);
    botWheelPID.setI(kI1);
    botWheelPID.setD(kD1);
    botWheelPID.setIZone(kIz1);
    botWheelPID.setFF(kFF1);
    botWheelPID.setOutputRange(kMinOutput, kMaxOutput);

    topWheelPID = shootTop.getPIDController();
    topWheelPID.setP(kP);
    topWheelPID.setI(kI);
    topWheelPID.setD(kD);
    topWheelPID.setIZone(kIz);
    topWheelPID.setFF(kFF);
    topWheelPID.setOutputRange(kMinOutput, kMaxOutput);

    sequencer = new CANSparkMax(54, MotorType.kBrushless);
    // sequencer = new CANSparkMax(46, MotorType.kBrushless);

    sequencer.setIdleMode(IdleMode.kCoast);
    sequenceEnc = new CANEncoder(sequencer);

    indexPID = sequencer.getPIDController();
    indexPID.setP(sP);
    indexPID.setI(sI);
    indexPID.setD(sD);
    indexPID.setIZone(sIz);
    indexPID.setFF(sFF);
    indexPID.setOutputRange(-0.5, 0.5);

    kicker = new WPI_VictorSPX(9);
    // kicker = new WPI_VictorSPX(7);

    climb = new WPI_TalonSRX(11);
    climb.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    climb.setNeutralMode(NeutralMode.Brake);
    // climb.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
    // LimitSwitchNormal.NormallyOpen);
    // climb.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
    // LimitSwitchNormal.NormallyOpen);

    // ControlPanel = new WPI_TalonSRX(8);
    // Intake = new WPI_VictorSPX(2);

    ControlPanel = new WPI_TalonSRX(10);
    Intake = new WPI_VictorSPX(13);

    spool = new WPI_TalonSRX(12);
    // spool = new WPI_TalonSRX(5); // Change device number

    drive = new DifferentialDrive(l1, r1);

    botWheelPID.setIAccum(0);
    topWheelPID.setIAccum(0);

    r1.configPeakCurrentLimit(30);
    l1.configPeakCurrentLimit(30);
    sequenceEnc.setPosition(0);

    ahrs.reset();

    in = new DigitalInput(0);
    //out = new DigitalInput(1);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

    // CHANGE THIS VARIABLE TO A NUMBER ABOVE 3
    // TO HAVE THE DEFAULT AUTO WHERE THE ROBOT DOESNT MOVE.
    // SET THE VARIABLE TO 1 IN ORDER TO RUN THE DRIVE FORWARD
    // AND SHOOT AUTONOMOUS.
    stage = 1;

    sequenceEnc.setPosition(0);
    r1.setNeutralMode(NeutralMode.Brake);
    l1.setNeutralMode(NeutralMode.Brake);
    r2.setNeutralMode(NeutralMode.Brake);
    l2.setNeutralMode(NeutralMode.Brake);
    tempRight = r1.getSelectedSensorPosition();
    tempLeft = l1.getSelectedSensorPosition();
    tempAutoShoot = sequenceEnc.getPosition();
    ahrs.reset();
    increment = true;
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    // BASIC AUTO FOR GULL LAKE

    switch (stage) {
    case 1:
      // TWEAK THE FIRST ARGUMENT TO ADJUST THE AMOUNT IT DRIVES FORWARD
      // TWEAK THE THIRD ARGUMENT TO ADJUST THE SPEED AT WHICH THE ROBOT DRIVES
      // FORWARD
      Functions.DriveTo(100000, false, 0.3, true, false);
      break;
    case 2:
      Functions.TargetAuton();
      break;
    case 3:
      // TWEAK THE FIRST TWO ARGUMENTS TO ALTER THE SHOOTER WHEEL SPEEDS
      // THE FIRST NUMBER CONTROLS THE BOTTOM WHEEL
      // THE SECOND NUMBER CONTROLS THE TOP WHEEL
      Functions.AutoShoot(3000, 2500, 0.5, false);
      break;
    case 4:
      Functions.suckin();
      Functions.DriveTo(150000, false, 0.3, true, false);
      break;

    case 5:
      Functions.TargetAuton();
      break;
    case 6:
      Functions.AutoShoot(3000, 2500, 0.5, false);
      break;

    default:
      r1.setNeutralMode(NeutralMode.Coast);
      l1.setNeutralMode(NeutralMode.Coast);
      r2.setNeutralMode(NeutralMode.Coast);
      l2.setNeutralMode(NeutralMode.Coast);
      break;
    }

    /*switch (stage) {
      case 1:
        // TWEAK THE FIRST ARGUMENT TO ADJUST THE AMOUNT IT DRIVES FORWARD
        // TWEAK THE THIRD ARGUMENT TO ADJUST THE SPEED AT WHICH THE ROBOT DRIVES
        // FORWARD
        Functions.DriveTo(100000, false, -0.3, true, false);
        break;
      case 2:
        // TWEAK THE FIRST TWO ARGUMENTS TO ALTER THE SHOOTER WHEEL SPEEDS
        // THE FIRST NUMBER CONTROLS THE BOTTOM WHEEL
        // THE SECOND NUMBER CONTROLS THE TOP WHEEL
        //Functions.AutoShoot(2500, 3202, 0.5, false);
          
        Functions.TargetAuton();
        break;
  
      default:
        r1.setNeutralMode(NeutralMode.Coast);
        l1.setNeutralMode(NeutralMode.Coast);
        r2.setNeutralMode(NeutralMode.Coast);
        l2.setNeutralMode(NeutralMode.Coast);
        break;
      }*/

    // COMPLICATED AUTO TO WORK ON FOR KENTWOOD AND ALPENA AND STATES
    /*
     * switch (m_autoSelected) { case KPortalAuto: switch (stage) { case 1:
     * Functions.DriveTo(100000, false, .7, increment, false);
     * Functions.TargetAuton(); break; case 2: Functions.AutoShoot(4400, 4400, 0.7,
     * true); break; case 3: Functions.DriveTo(180000, true, 1, increment, true);
     * Functions.TargetAuton(); break; case 4: Functions.AutoShoot(4400, 4400, 0.2,
     * true); break; default: Intake.set(0); r1.setNeutralMode(NeutralMode.Coast);
     * l1.setNeutralMode(NeutralMode.Coast); r2.setNeutralMode(NeutralMode.Coast);
     * l2.setNeutralMode(NeutralMode.Coast); break; } break; case kDriveShoot:
     * switch (stage) { case 1: //Functions.DriveTo(100000, false, 0.3, true,
     * false); // Functions.TargetAuton(); r1.set(ControlMode.Position, tempRight +
     * 7000); l1.set(ControlMode.Position, tempLeft + 7000); break; default:
     * r1.setNeutralMode(NeutralMode.Coast); l1.setNeutralMode(NeutralMode.Coast);
     * r2.setNeutralMode(NeutralMode.Coast); l2.setNeutralMode(NeutralMode.Coast);
     * break; } break; case kDefaultAuto: default:
     * r1.setNeutralMode(NeutralMode.Coast); l1.setNeutralMode(NeutralMode.Coast);
     * r2.setNeutralMode(NeutralMode.Coast); l2.setNeutralMode(NeutralMode.Coast);
     * break; }
     */
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    if (drStick.getRawButton(11)) {
      // ahrs.reset();
      shootTurret.setSelectedSensorPosition(0);
      sequenceEnc.setPosition(0);
      climb.setSelectedSensorPosition(0);

    }
    // This gets all the numbers we need from the smart dashboard
    distoff = SmartDashboard.getNumber("distance off", 0);
    Visionclass = SmartDashboard.getString("class", "");
    size = SmartDashboard.getNumber("size", 0);
    midx1 = SmartDashboard.getNumber("mid x", 0);
    tensortime = SmartDashboard.getNumber("time", 0);
    time = SmartDashboard.getNumber("time", 0);

    // Specials stuff

    // Vision target, if the button is pressed vision kicks in
    if (spStick.getRawButton(3) || spStick.getRawButton(5)) {
      // Call Vision teleop function
      Functions.TargetTel();

    }

    // If the button isnt pressed you can control it
    else {

      // if its inbetween the two positions you can move it either way or you hit
      // button 6 or 7
      if ((shootTurret.getSelectedSensorPosition() < lStop && shootTurret.getSelectedSensorPosition() > rStop)
          || (spStick.getRawButton(7) || spStick.getRawButton(8))) {

        shootTurret.set(-spStick.getZ() * 0.3);

      }

      // If it hit one of the stops you can only move one way
      else if (shootTurret.getSelectedSensorPosition() < rStop || shootTurret.getSelectedSensorPosition() > lStop) {

        // if it hit the r stop you can only move left
        if (shootTurret.getSelectedSensorPosition() <= rStop) {

          if (spStick.getZ() < 0) {
            shootTurret.set(spStick.getZ() * 0.3);
          }

          else {
            shootTurret.set(0);
          }

        }

        // if it hit the l stop you can only move right
        else if (shootTurret.getSelectedSensorPosition() >= lStop) {

          if (spStick.getZ() > 0) {
            shootTurret.set(spStick.getZ() * 0.3);
          }

          else {
            shootTurret.set(0);
          }

        }

      }

    }

    //count the number of balls
    if(!in.get()){
      gotball = true;
    }
    else if(climbStick.getRawButton(7)){
      ballcount = 0;
      gotball = false;
    }

    if(gotball == true && in.get()){
      ballcount++;
      gotball = false;
    }

    /*else if(out.get()){
      ballcount = ballcount - 1;
    }*/

    // Shooter rev up
    if (spStick.getRawButton(1)) {
      shootTurret.set(0);
      if (shoot) {
        tempShoot = sequenceEnc.getPosition() - sequenceEnc.getPosition() % 8.5;
        shoot = false;
      }
      if (!(sequenceEnc.getPosition() % 10.5 <= 0.1) && !(sequencer.get() < 0.1 && sequencer.get() > 0.1)) {
        indexPID.setReference(tempShoot, ControlType.kPosition);
      } else {
        indexPID.setReference(tempShoot, ControlType.kPosition);
        kicker.set(-1);
      }
      // 0.88
      botWheelPID.setReference(3000, ControlType.kVelocity);
      topWheelPID.setReference(2500, ControlType.kVelocity);
      //shootTop.set(1);
      //shootBottom.set(1);
      // shootTop.set(1);
      // shootBottom.set(1);
      // shootTurret.set(0);

      if (drStick.getRawButton(4)) {
        tempShoot += 0.7;
      }

      if (shootTopEnc.getVelocity() > 2450 && shootBottomEnc.getVelocity() > 2950) {
        tempShoot += 0.5;
      }

    } else {
      // kicker.set(0);
      shootTop.set(0);
      shootBottom.set(0);
      shoot = true;
    }

    // ball stuck get out of da robot you goofy goober
    if (spStick.getRawButton(12) || spStick.getRawButton(11)) {
      kicker.set(0.2);
      Intake.set(0.5);
      sequencer.set(-0.2);
    } else {
      // kicker.set(0);
      Intake.set(0);
    }

    if (spStick.getRawButton(10)) {
      kicker.set(.2);
    }

    // intake the ball - R.I.P Scoopy motor :(
    if (spStick.getRawButton(2)) {
      Intake.set(-1);
      sequencer.set(0.2);
    }
    // outtake the ball
    else if (spStick.getRawButton(9)) {
      Intake.set(.5);
    } else {
      Intake.set(0);
    }

    if (spStick.getRawButton(4)) {
      climb.set(.5);
    } else if (spStick.getRawButton(6)) {
      climb.set(-.5);
    } else {
      climb.set(0);
    }

    // turn the sequencer off
    if (!spStick.getRawButton(2) && !spStick.getRawButton(12) && !spStick.getRawButton(11)
        && !spStick.getRawButton(1)) {
      sequencer.set(0);
    }

    if (!spStick.getRawButton(10) && !spStick.getRawButton(12) && !spStick.getRawButton(11)
        && !spStick.getRawButton(1)) {
      kicker.set(0);
    }

    if (climbStick.getY() > .2) {
      climb.set(climbStick.getY());
    } 
    else if (climbStick.getY() < .2) {
      climb.set(climbStick.getY());
    } 
    else {
      climb.set(0);
    }

    // Driver Stuff

    if (drStick.getRawButton(2)) {
      ControlPanel.set(0.85);
    } else {
      ControlPanel.set(0);
    }

    // Spool to pick up the arm so we fit ;)
    if (drStick.getRawButton(3)) {
      spool.set(.5);
    } else if (drStick.getRawButton(5)) {
      spool.set(-.5);
    } else {
      spool.set(0);
    }

    // If you press 1 you vision good
    if (drStick.getRawButton(1)) {
      SmartDashboard.putNumber("cam", 1);

      // Make sure it has a ball
      /*if (Visionclass.compareTo("ball") >= 0) {

        Functions.TurnBallVision();

        Functions.SpeedBallVision();

        // actually drive to the robot
        drive.arcadeDrive(Speedvalball, Turnvalball);
      }*/
    }
    // if your not activating vision you can drive nic and corey
    else {
      SmartDashboard.putNumber("cam", 0);
      if (spStick.getRawButton(1)) {
        drive.arcadeDrive(0, 0);
      } else {
        drive.arcadeDrive(drStick.getY(), -drStick.getZ());
      }
    }

    // puts any nessacary values on the smart dashboard
    SmartDashboard.putNumber("ShootEnc", shootTurret.getSelectedSensorPosition());
    SmartDashboard.putNumber("Shoot top enc", shootTopEnc.getPosition());
    SmartDashboard.putNumber("Shoot top current", shootTop.getOutputCurrent());
    SmartDashboard.putNumber("Shoot bot enc", shootBottomEnc.getPosition());
    SmartDashboard.putNumber("Shoot bot current", shootBottom.getOutputCurrent());
    SmartDashboard.putNumber("Speedval", Speedvalball);
    SmartDashboard.putNumber("Turn Value", Turnvalball);
    SmartDashboard.putNumber("midx 1", midx1);
    SmartDashboard.putNumber("midx 2", midx2);
    SmartDashboard.putNumber("TopShootVelocity", shootTopEnc.getVelocity());
    SmartDashboard.putNumber("BottomShootVelocity", shootBottomEnc.getVelocity());
    SmartDashboard.putNumber("Drum", sequenceEnc.getPosition());
    SmartDashboard.putNumber("left1Enc", l1.getSelectedSensorPosition());
    SmartDashboard.putNumber("right1Enc", r1.getSelectedSensorPosition());
    SmartDashboard.putNumber("Angle", ahrs.getAngle());
    SmartDashboard.putBoolean("Got Target?", gottar);
    SmartDashboard.putNumber("ShootTop", shootTop.getAppliedOutput());
    SmartDashboard.putNumber("ShootBottom", shootBottom.getAppliedOutput());
    SmartDashboard.putNumber("climb", climb.getSelectedSensorPosition());
    SmartDashboard.putNumber("Match Time", DriverStation.getInstance().getMatchTime());
    SmartDashboard.putNumber("Ball Count", ballcount);

    // calculates the timer for the drive and prediction
    if (t < 51) {
      t++;
    } else {
      i = true;
    }

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
