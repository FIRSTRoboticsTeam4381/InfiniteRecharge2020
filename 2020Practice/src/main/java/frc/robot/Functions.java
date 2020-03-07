package frc.robot;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Functions {

    public static Robot robot;

    public static void DriveTo(int pos, boolean pickBall, double speed, boolean inc, boolean modSpeed) {
        if(Math.abs(Math.abs(robot.l1.getSelectedSensorPosition()) - Math.abs(robot.tempLeft)) < Math.abs(pos)
         && Math.abs(Math.abs(robot.r1.getSelectedSensorPosition()) - Math.abs(robot.tempRight)) < Math.abs(pos)){
           if(pickBall){
               robot.Intake.set(-1);
               robot.sequencer.set(0.2);
           }
           if(modSpeed){
            robot.r1.set((speed*((Math.abs(robot.tempLeft) - Math.abs(robot.l1.getSelectedSensorPosition())) / robot.tempLeft))+0.2);
            robot.l1.set(-(speed*((Math.abs(robot.tempRight) - Math.abs(robot.r1.getSelectedSensorPosition())) / robot.tempRight))+0.2);
           }else{
            robot.r1.set(speed);
            robot.l1.set(-speed);
           }
           
         }else{
            robot.sequencer.set(0);
            robot.r1.set(0);
            robot.l1.set(0);
            robot.tempLeft = robot.l1.getSelectedSensorPosition();
            robot.tempRight = robot.r1.getSelectedSensorPosition();
            robot.tempAutoShoot = robot.sequenceEnc.getPosition();
            if(inc){
              robot.stage++; 
            }
         }
      }
    
      public static void AutoShoot(double sbVel, double stVel, double indexVel, boolean time) {
        if(time){  
          robot.indexPID.setReference(119, ControlType.kPosition);
        }

        robot.botWheelPID.setReference(1, ControlType.kDutyCycle);
        robot.topWheelPID.setReference(1, ControlType.kDutyCycle);
        if(robot.sequenceEnc.getPosition() % 8.5 < 0.05){
        robot.kicker.set(-1);
        }
        if(robot.shootBottomEnc.getVelocity() < sbVel && robot.shootTopEnc.getVelocity() < stVel){
          
        }else{
            if(robot.sequenceEnc.getPosition() < 59.5 + robot.tempAutoShoot){
               robot.sequencer.set(indexVel);
            }else{
                robot.sequencer.set(0);
                robot.shootTop.set(0);
                robot.shootBottom.set(0);
                robot.kicker.set(0);
                robot.tempAutoShoot = robot.sequenceEnc.getPosition();
                robot.stage++;
            }
    }
    }

    public static void TurnTo(int deg){
        
            if(robot.ahrs.getAngle() > deg + 3){
                robot.l1.set(0.2);
                robot.r1.set(0.2);
            }else if(robot.ahrs.getAngle() < deg - 3){
                robot.l1.set(-0.2);
                robot.r1.set(-0.2);
            }else{
                robot.l1.set(0);
                robot.r1.set(0);
                robot.tempLeft = robot.l1.getSelectedSensorPosition();
                robot.tempRight = robot.r1.getSelectedSensorPosition();
                robot.stage++;
            }
    }

    public static void TurnBallVision(){
        //calulates the turn value of the robot
        if(robot.distoff > 0){
            //subtract the offset - Make sure you have the correct value!!!
            robot.distoff = robot.distoff - robot.offsetBall;
          }
  
          else{
            robot.distoff = robot.distoff + robot.offsetBall;
          }
    
          if(robot.midx1 == 0){
            robot.midx2 = 0;
            robot.i = false;
            robot.t = 0;
          }
          else{
            robot.midx2 = robot.midx1 - robot.midx2; 
          }
    
          if(robot.size > robot.sizeCheck && robot.size != 0){
            if(robot.distoff < 270 && robot.distoff > 370){
              robot.Turnvalball = 0;
            }
  
            else{
                robot.Turnvalball = (0.003125 * robot.distoff);  
                robot.Turnvalball = robot.Turnvalball * 0.6;
            }
  
          }
  
          else if(robot.size < 200 && robot.size != 0){
  
            if(robot.distoff < 300 && robot.distoff > 340){
                robot.Turnvalball = 0;
            }
  
            else{
              if(robot.midx2 > 20 || robot.midx2 < -20){
                robot.Turnvalball = (0.003125 * robot.distoff);  
                robot.Turnvalball = robot.Turnvalball * .8;
                robot.Turnvalball = robot.Turnvalball * -1;
              }
  
              else if(robot.t > 51){
                robot.Turnvalball = (0.003125 * robot.distoff);  
                robot.Turnvalball = robot.Turnvalball * 0.6;
              }
            }
          }
          else{
            robot.Turnvalball = 0;
          }
    }

    public static void SpeedBallVision(){
        //Calulates the speed value of the robot
        if(robot.size == 0){
            robot.Speedvalball = 0;
          }
          else{
            if(robot.t < 51){
              robot.Speedvalball = ((-0.003125) * robot.size) + 1;
              robot.Speedvalball = robot.Speedvalball * 0.85;
            }
            else{
              robot.Speedvalball = ((-0.003125) * robot.size) + 1;
            }
          }
    }

    public static void TargetTel(){

      if(robot.distoff > 0){
          //subtract the offset - Make sure you have the correct value!!!
          robot.distoff = robot.distoff - robot.offsetTar;
      }

      else{
          robot.distoff = robot.distoff + robot.offsetTar;
      }

      if(robot.Visionclass.compareTo("Target") >= 0){
        if(robot.size > robot.sizeCheck){
          if(robot.distoff > 10 || robot.distoff < -10){
            robot.gottar = false;
          
            robot.Turnvaltar = (0.003125 * robot.distoff);  
            robot.Turnvaltar = robot.Turnvaltar * 0.25;
  
            if(robot.Turnvaltar > 0 && robot.Turnvaltar < .1){
              robot.Turnvaltar = .05;
            }
            if(robot.Turnvaltar < 0 && robot.Turnvaltar > -.1){
              robot.Turnvaltar = -.05;
            }
  
          }
          else{
            robot.Turnvaltar = 0;
            robot.gottar = true;
          }
        }
        else if(robot.size <= robot.sizeCheck){
          if(robot.distoff >= 5 || robot.distoff <= -5){
            robot.gottar = false;
          
            robot.Turnvaltar = (0.003125 * robot.distoff);  
            robot.Turnvaltar = robot.Turnvaltar * 0.25;
  
            if(robot.Turnvaltar > 0 && robot.Turnvaltar < .1){
              robot.Turnvaltar = .05;
            }
            if(robot.Turnvaltar < 0 && robot.Turnvaltar > -.1){
              robot.Turnvaltar = -.05;
            }
  
          }
          else{
            robot.gottar = true;
            robot.Turnvaltar = 0;
          }
        }
      }
      else{
        robot.Turnvaltar = 0;
        robot.gottar = false;
        //robot.Turnvaltar = 0;
      
  
        /*if(robot.shootTurret.getSelectedSensorPosition() >= robot.lStop && robot.Turnvaltar > 0){
          robot.Turnvaltar = 0;
          robot.searchspeedTel = -Math.abs(robot.searchspeedTel);
        }
        if(robot.shootTurret.getSelectedSensorPosition() <= robot.rStop && robot.Turnvaltar < 0){
          robot.Turnvaltar = 0;
          robot.searchspeedTel = Math.abs(robot.searchspeedTel);
        }*/
      }
      robot.shootTurret.set(-robot.Turnvaltar);
    } 

    
    
    public static void TargetAuton(){
      robot.Visionclass = SmartDashboard.getString("class", "");
      SmartDashboard.putNumber("ShootEnc", robot.shootTurret.getSelectedSensorPosition());
      SmartDashboard.putNumber("TurnVal", robot.Turnvaltar);
      robot.distoff = SmartDashboard.getNumber("distance off", 0);

      if(robot.distoff > 0){
        //subtract the offset - Make sure you have the correct value!!!
        robot.distoff = robot.distoff - robot.offsetTar;
      }

      else{
        robot.distoff = robot.distoff + robot.offsetTar;
      }

      if(robot.Visionclass.compareTo("Target") >= 0){
        if(robot.distoff >= 10 || robot.distoff <= -10){
        
          robot.Turnvaltar = (0.003125 * robot.distoff);  
          robot.Turnvaltar = robot.Turnvaltar * 0.5;

          if(robot.Turnvaltar > 0 && robot.Turnvaltar < .1){
            robot.Turnvaltar = .1;
          }
          if(robot.Turnvaltar < 0 && robot.Turnvaltar > -.1){
            robot.Turnvaltar = -.1;
          }

        }
        else{
          robot.Turnvaltar = 0;
          RevEmUp(1);
          robot.increment = true;
        }
      }
      else{
         robot.Turnvaltar = robot.searchspeed;
      }

      if(robot.shootTurret.getSelectedSensorPosition() >= robot.lStop && robot.Turnvaltar > 0){
        robot.Turnvaltar = 0;
        robot.searchspeed = -Math.abs(robot.searchspeed);
      }
      if(robot.shootTurret.getSelectedSensorPosition() <= robot.rStop && robot.Turnvaltar < 0){
        robot.Turnvaltar = 0;
        robot.searchspeed = Math.abs(robot.searchspeed);
      }
      robot.shootTurret.set(-robot.Turnvaltar);
    }

    public static void RevEmUp(double FreedomsPerYeeHaw){
      robot.botWheelPID.setReference(FreedomsPerYeeHaw, ControlType.kDutyCycle);
      robot.topWheelPID.setReference(FreedomsPerYeeHaw, ControlType.kDutyCycle);
    }
    


}