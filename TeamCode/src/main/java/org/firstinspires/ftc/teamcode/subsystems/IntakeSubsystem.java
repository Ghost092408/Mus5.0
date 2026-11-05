package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
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
      
      intake.setInverted(true);
   }
   
   public void suge() {
      intake.set(1);
      transfer.set(1);
   }
   
   public void stop() {
      intake.set(1);
      transfer.set(1);
   }
}
