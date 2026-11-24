package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Configurable
public class TurretSubsystem {
   private final MotorEx flywheel;
   
   private final ServoEx limba;
   
   public double targetVelocity = 0;
   
   public TurretSubsystem(HardwareMap hardwareMap) {
      flywheel = new MotorEx(hardwareMap, "turret");
      
      limba = new ServoEx(hardwareMap, "rampAng");
      
      flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
      
      flywheel.setRunMode(Motor.RunMode.RawPower);
      
      flywheel.setInverted(true);
   }
   
   public void setSpeed(double ticksPerSecond) {
      targetVelocity = ticksPerSecond;
   }
   
   public void setPower(double pow) { flywheel.set(pow); }
   
   public double getVelocity() {
      return flywheel.getVelocity();
   }
   
   public boolean atSpeed() {
      return targetVelocity > 100 && Math.abs(targetVelocity - flywheel.getVelocity()) < 35;
   }
   
   public void stop() { setSpeed(0); }
   
   public void setAngle(double x) { limba.set(x); }
}
