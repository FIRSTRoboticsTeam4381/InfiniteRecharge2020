package frc.robot;
import com.revrobotics.ControlType;

public class Functions {

    public static Robot robot;

    public static void DriveTo(int pos, boolean pickBall) {
        if(Math.abs(Math.abs(robot.l1.getSelectedSensorPosition()) - Math.abs(robot.tempLeft)) < pos
         && Math.abs(Math.abs(robot.r1.getSelectedSensorPosition()) - Math.abs(robot.tempRight)) < pos){
           if(pickBall){
               robot.Intake.set(-1);
               robot.sequencer.set(0.2);
           }
           robot.r1.set(0.2);
           robot.l1.set(-0.2);
         }else{
            robot.Intake.set(0);
            robot.sequencer.set(0);
            robot.r1.set(0);
            robot.l1.set(0);
            robot.stage++; 
         }
      }
    
      public static void AutoShoot(double sbVel, double stVel, double indexVel) {
        
        robot.botWheelPID.setReference(1, ControlType.kDutyCycle);
        robot.topWheelPID.setReference(1, ControlType.kDutyCycle);
        robot.kicker.set(-1);
        if(robot.shootBottomEnc.getVelocity() < sbVel && robot.shootTopEnc.getVelocity() < stVel){
            //Wait
        }else{
            if(robot.sequenceEnc.getPosition() < 68){
                robot.sequencer.set(indexVel);
            }else{
                robot.sequencer.set(0);
                robot.shootTop.set(0);
                robot.shootBottom.set(0);
                robot.kicker.set(0);
                robot.stage++;
            }
    }
    }

    public static void TurnTo(int deg){
        
            if(robot.ahrs.getAngle() > deg + 3){
                robot.l1.set(0.3);
                robot.r1.set(0.3);
            }else if(robot.ahrs.getAngle() < deg - 3){
                robot.l1.set(-0.3);
                robot.r1.set(-0.3);
            }else{
                robot.l1.set(0);
                robot.r1.set(0);
                robot.stage++;
            }
    }

    public static void inbetweenTarget(){
        if(robot.size > 200 && robot.size != 0){

            if(robot.distoff < 270 && robot.distoff > 370){
              robot.Turnvaltar = 0;
            }

            else{
                robot.Turnvaltar = (0.003125 * robot.distoff);  
                robot.Turnvaltar = robot.Turnvaltar * 0.3;
            }

          }
          else if(robot.size < 200 && robot.size != 0){

            if(robot.distoff < 270 && robot.distoff > 370){
                robot.Turnvaltar = 0;
            }

            else{
                robot.Turnvaltar = (0.003125 * robot.distoff);  
                robot.Turnvaltar = robot.Turnvaltar * 0.3;
              }

          }
    } 
    
    public static void pastRightTarget(){
        if(robot.Turnvaltar > 0){

            if(robot.size > 200 && robot.size != 0){

              if(robot.distoff < 270 && robot.distoff  > 370){
                robot.Turnvaltar = 0;
              }
              else{
                robot.Turnvaltar = (0.003125 * robot.distoff );  
                robot.Turnvaltar = robot.Turnvaltar * 0.4;
              }

            }

            else if(robot.size < 200 && robot.size != 0){

              if(robot.distoff  < 270 && robot.distoff  > 370){
                robot.Turnvaltar = 0;
              }

            else{
                robot.Turnvaltar = (0.003125 * robot.distoff);  
                robot.Turnvaltar = robot.Turnvaltar * 0.4;
              }

            }
          }

        else{
                robot.Turnvaltar = 0;
            }
    }

    public static void pastLeftTarget(){
        if(robot.Turnvaltar < 0){

            if(robot.size > 200 && robot.size != 0){

              if(robot.distoff < 250 && robot.distoff > 390){
                robot.Turnvaltar = 0;
              }

              else{
                robot.Turnvaltar = (0.003125 * robot.distoff);  
                robot.Turnvaltar = robot.Turnvaltar * 0.4;
              }

            }

            else if(robot.size < 200 && robot.size != 0){

              if(robot.distoff < 270 && robot.distoff > 370){
                robot.Turnvaltar = 0;
              }

              else{
                robot.Turnvaltar = (0.003125 * robot.distoff);  
                robot.Turnvaltar = robot.Turnvaltar * 0.4;
              }

            }

          }
          else{
            robot.Turnvaltar = 0;
          }
    }

    public static void TurnBallVision(){
        //calulates the turn value of the robot
        if(robot.distoff > 0){
            //subtract the offset - Make sure you have the correct value!!!
            robot.distoff = robot.distoff - robot.offset;
          }
  
          else{
            robot.distoff = robot.distoff + robot.offset;
          }
    
          if(robot.midx1 == 0){
            robot.midx2 = 0;
            robot.i = false;
            robot.t = 0;
          }
          else{
            robot.midx2 = robot.midx1 - robot.midx2; 
          }
    
          if(robot.size > 200 && robot.size != 0){
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

    public static void TargetAuton(){
        robot.stage++;
    }

    public static void TargetTel(){
        
    }

    public static void TeleShoot(){
        robot.indexPID.setReference(8.5, ControlType.kPosition);
    }
    


}