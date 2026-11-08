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
   
//   private final ServoEx turretAng;
//   private final ServoEx limba;
   
   public static double kP = 0;
   public static double kI = 0;
   public static double kD = 0;
   public static double kF = 0;
   
   private final PIDFController velocityController = new PIDFController(kP, kI, kD, kF);
   
   private double targetVelocity = 0;
   
   public TurretSubsystem(HardwareMap hardwareMap) {
      flywheel = new MotorEx(hardwareMap, "turret");
      
//      turretAng = new ServoEx(hardwareMap, "turretAng");
//      limba = new ServoEx(hardwareMap, "limba");
      
      flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
      
      flywheel.setRunMode(Motor.RunMode.RawPower);
   }
   
   public void setSpeed(double ticksPerSecond) {
      targetVelocity = ticksPerSecond;
   }
   
   public double getVelocity() {
      return flywheel.getVelocity();
   }
   
   public boolean atSpeed() {
      return Math.abs(targetVelocity - flywheel.getVelocity()) < 10;
   }
   
   public void stop() {
      setSpeed(0);
      update();
   }
   
   public void update() {
      if (targetVelocity == 0) {
         velocityController.reset();
         flywheel.set(0);
         return;
      }
      velocityController.setPIDF(kP, kI, kD, kF);
      double power = velocityController.calculate(flywheel.getVelocity(), targetVelocity);
      flywheel.set(Range.clip(power, 0, 1));
   }
}
