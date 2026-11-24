package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class IntakeSubsystem extends SubsystemBase {
   private final MotorEx intake;
   private final MotorEx transfer;
   
   public IntakeSubsystem(HardwareMap hardwareMap) {
      intake = new MotorEx(hardwareMap, "intake");
      transfer = new MotorEx(hardwareMap, "transfer");
      
      intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
      transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
      
      intake.setRunMode(Motor.RunMode.RawPower);
      transfer.setRunMode(Motor.RunMode.RawPower);
      
      transfer.setInverted(true);
   }
   
   public void setPowInt (double pow) { intake.set(pow); }
   
   public void setPowTrans (double pow) { transfer.set(pow); }
   
   public void stop() {
      intake.set(0);
      transfer.set(0);
   }
}
