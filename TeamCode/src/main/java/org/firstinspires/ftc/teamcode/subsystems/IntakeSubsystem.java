package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class IntakeSubsystem extends SubsystemBase {
   private final MotorEx intake;
   
   public IntakeSubsystem(HardwareMap hardwareMap) {
      intake = new MotorEx(hardwareMap, "intake");
      intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
      intake.setRunMode(Motor.RunMode.RawPower);
      intake.setInverted(true);
   }
   
   public void suge() { intake.set(1); }
   
   public void stop() { intake.set(0); }
}
