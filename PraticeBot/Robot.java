/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private CANSparkMax sequencer;
  //private SparkMax HandleExtension;
  private CANSparkMax shootTop;
  private CANSparkMax shootBottom;
  //private SparkMax HandleDrive;

  private WPI_VictorSPX kicker;
  private WPI_VictorSPX Intake;
  private WPI_VictorSPX r2;
  private WPI_VictorSPX l2;

  //private WPI_TalonSRX ControlPanel;
  private WPI_TalonSRX r1;
  private WPI_TalonSRX l1;
  //private WPI_TalonSRX PickupArm;
 //
  private WPI_TalonSRX shootTurret;

  private WPI_TalonSRX spool;

  private double distoff = 0;
  private double size = 0;
  private double midx1 = 0;
  private double midx2 = 0;
  private double tensortime = 0;

  private int t = 0;
  private boolean i = false;

  private double offset = 75; //Change Offsett value

  private String Visionclass = "";

  private double Turnvalball = 0;
  private double Speedvalball = 0;

  private double camnum = 0;

  private double Turnvaltar = 0;
  private double Speedvaltar = 0;

  private Joystick drStick = new Joystick(0);
  private Joystick spStick = new Joystick(1);
  private DifferentialDrive drive;

  private int lStop = 23396; //CHANGE LATER
  private int rStop = -11855; //CHANGE LATER

  /**1
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    r1 = new WPI_TalonSRX(6);
    r2 = new WPI_VictorSPX(3);
    l1 = new WPI_TalonSRX(4);
    l2 = new WPI_VictorSPX(1);

    r2.follow(r1);
    l2.follow(l1);

    shootTurret = new WPI_TalonSRX(9);
    shootTurret.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    shootTurret.setNeutralMode(NeutralMode.Brake);
    shootTop = new CANSparkMax(45, MotorType.kBrushless);
    shootBottom = new CANSparkMax(53, MotorType.kBrushless);
    
    kicker = new WPI_VictorSPX(7);
    sequencer = new CANSparkMax(46, MotorType.kBrushless);
    sequencer.setIdleMode(IdleMode.kCoast);

    //PickupArm = new WPI_TalonSRX(1);
    Intake = new WPI_VictorSPX(2);

    spool = new WPI_TalonSRX(5); //Change device number

    drive = new DifferentialDrive(l1, r1);
   

  }


  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    distoff = SmartDashboard.getNumber("distance off", 0);
    Visionclass = SmartDashboard.getString("class", "");
    size = SmartDashboard.getNumber("size", 0);
    midx1 = SmartDashboard.getNumber("mid x", 0);
    tensortime = SmartDashboard.getNumber("time", 0);

    //Specials stuff

    //Vision target
    if(spStick.getRawButton(7)){
      camnum = 1;
      if(Visionclass.compareTo("Target") >= 0){
        if((shootTurret.getSelectedSensorPosition() < lStop && shootTurret.getSelectedSensorPosition() > rStop)){
          if(size > 200 && size != 0){
            if(distoff < 270 && distoff > 370){
              Turnvaltar = 0;
            }
            else{
              Turnvaltar = (0.003125 * distoff);  
              Turnvaltar = Turnvaltar * 0.4;
              //Turnvalball = (0.005 * distoff); 
              //Turnvalball = Math.copySign(Math.min(.8, Math.abs(Turnvalball)), Turnvalball);
            }
          }
          else if(size < 200 && size != 0){
            if(distoff < 270 && distoff > 370){
              Turnvaltar = 0;
            }
            else{
                Turnvaltar = (0.003125 * distoff);  
                Turnvaltar = Turnvaltar * 0.4;
              }
          }
        else{
          Turnvaltar = 0;
        }
      }
      else if(shootTurret.getSelectedSensorPosition() <= rStop){
          if(Turnvaltar > 0){
            if(size > 200 && size != 0){
              if(distoff < 270 && distoff > 370){
                Turnvaltar = 0;
            }
              else{
                Turnvaltar = (0.003125 * distoff);  
                Turnvaltar = Turnvaltar * 0.4;
                //Turnvalball = (0.005 * distoff); 
                //Turnvalball = Math.copySign(Math.min(.8, Math.abs(Turnvalball)), Turnvalball);
              }
            }
            else if(size < 200 && size != 0){
              if(distoff < 270 && distoff > 370){
                Turnvaltar = 0;
              }
              else{
                  Turnvaltar = (0.003125 * distoff);  
                  Turnvaltar = Turnvaltar * 0.4;
                }
            }
          else{
            Turnvaltar = 0;
            }
          }
          else{
            Turnvaltar = 0;
          }
        }
        else if(shootTurret.getSelectedSensorPosition() >= lStop){
          if(Turnvaltar < 0){
            if(size > 200 && size != 0){
              if(distoff < 270 && distoff > 370){
                Turnvaltar = 0;
            }
              else{
                Turnvaltar = (0.003125 * distoff);  
                Turnvaltar = Turnvaltar * 0.4;
                //Turnvalball = (0.005 * distoff); 
                //Turnvalball = Math.copySign(Math.min(.8, Math.abs(Turnvalball)), Turnvalball);
              }
            }
            else if(size < 200 && size != 0){
              if(distoff < 270 && distoff > 370){
                Turnvaltar = 0;
              }
              else{
                  Turnvaltar = (0.003125 * distoff);  
                  Turnvaltar = Turnvaltar * 0.4;
                }
            }
          else{
            Turnvaltar = 0;
            }
          }
          else{
            Turnvaltar = 0;
          }
        }
        shootTurret.set(-Turnvaltar);
      }
    }
    else{
      if((shootTurret.getSelectedSensorPosition() < lStop && shootTurret.getSelectedSensorPosition() > rStop) || spStick.getRawButton(6)){
     
        shootTurret.set(-spStick.getZ() *0.3);
      }
      else if(shootTurret.getSelectedSensorPosition() < rStop || shootTurret.getSelectedSensorPosition() > lStop){
        if(shootTurret.getSelectedSensorPosition() <= rStop){
          if(spStick.getZ() < 0){
            shootTurret.set(spStick.getZ() *0.3);
          }
          else{
            shootTurret.set(0);
          }
        }
        else if(shootTurret.getSelectedSensorPosition() >= lStop){
          if(spStick.getZ() > 0){
            shootTurret.set(spStick.getZ() *0.3);
          }
          else{
            shootTurret.set(0);
          }
        }
      }
    }


    //Shooter and scoopy stuff
    if(spStick.getRawButton(3) || spStick.getRawButton(5)){
      kicker.set(-1);
      shootBottom.set(1);
      shootTop.set(1);
    }else if(spStick.getRawButton(12)){
      kicker.set(0.2);
    }else{
      kicker.set(0);
      shootBottom.set(0);
      shootTop.set(0);
    }
    if(spStick.getRawButton(1)){
      sequencer.set(0.5);
    }

    if(spStick.getRawButton(8)){
      spool.set(.5);
    }
    else{
      spool.set(0);
    }

    if(spStick.getRawButton(2)){
      Intake.set(-1);
      sequencer.set(0.2);
    }else if(spStick.getRawButton(11)){
      Intake.set(0.5);
      sequencer.set(-0.2);
    }else{
      Intake.set(0);
    }


    if(!spStick.getRawButton(3) && !spStick.getRawButton(5) && !spStick.getRawButton(2) && !spStick.getRawButton(11)){
      sequencer.set(0);
    }


    //Driver Stuff
    if(drStick.getRawButton(1)){
      camnum = 0;
      if(Visionclass.compareTo("ball") >= 0){
        if(distoff > 0){
          distoff = distoff - offset;
        }
        else{
          distoff = distoff + offset;
        }
  
        if(midx1 == 0){
          midx2 = 0;
          i = false;
          t = 0;
        }
        else{
          midx2 = midx1 - midx2; 
        }
  
        if(size > 200 && size != 0){
          if(distoff < 270 && distoff > 370){
            Turnvalball = 0;
          }
          else{
            Turnvalball = (0.003125 * distoff);  
            Turnvalball = Turnvalball * 0.6;
          //Turnvalball = (0.005 * distoff); 
          //Turnvalball = Math.copySign(Math.min(.8, Math.abs(Turnvalball)), Turnvalball);
          }
        }
        else if(size < 200 && size != 0){
          if(distoff < 280 && distoff > 360){
            Turnvalball = 0;
          }
          else{
            if(midx2 > 20 || midx2 < -20){
              Turnvalball = (0.003125 * distoff);  
              Turnvalball = Turnvalball * .8;
              Turnvalball = Turnvalball * -1;
            }
            else if(t > 51){
              Turnvalball = (0.003125 * distoff);  
              Turnvalball = Turnvalball * 0.6;
            }
          
            /*if(midx2 == 0){
              Turnvalball = (0.003125 * distoff);  
              Turnvalball = Turnvalball * 0.6;
            }*/
          
          //Turnvalball = (0.005 * distoff); 
          //Turnvalball = Math.copySign(Math.min(.8, Math.abs(Turnvalball)), Turnvalball);
          }
        }
        else{
          Turnvalball = 0;
        }
  
        if(size == 0){
          Speedvalball = 0;
        }
        else{
          if(t < 51){
            Speedvalball = ((-0.003125) * size) + 1;
            Speedvalball = Speedvalball * 0.85;
          }
          else{
            Speedvalball = ((-0.003125) * size) + 1;
          }
        }
        drive.arcadeDrive(Speedvalball, Turnvalball);
     //drive.arcadeDrive(0, -Turnvalball);
    }
  }
  else{
    drive.arcadeDrive(drStick.getY(), -drStick.getZ());
  }
    
    SmartDashboard.putNumber("ShootEnc", shootTurret.getSelectedSensorPosition());
    SmartDashboard.putNumber("Speedval", Speedvalball);
    SmartDashboard.putNumber("Turn Value", Turnvalball);
    SmartDashboard.putNumber("midx 1", midx1);
    SmartDashboard.putNumber("midx 2", midx2);
    SmartDashboard.putNumber("Cam", camnum);

    if(t < 51){
      t++;
    }
    else{
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

