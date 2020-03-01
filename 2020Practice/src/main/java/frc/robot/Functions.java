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

    public static void Target(){
        robot.stage++;
    }

    public static void TeleShoot(){
        robot.indexPID.setReference(8.5, ControlType.kPosition);
    }
    


}