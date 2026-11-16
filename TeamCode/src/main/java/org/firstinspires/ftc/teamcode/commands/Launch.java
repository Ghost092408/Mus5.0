package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.teleOps.debug.Test.kD;
import static org.firstinspires.ftc.teamcode.teleOps.debug.Test.kF;
import static org.firstinspires.ftc.teamcode.teleOps.debug.Test.kI;
import static org.firstinspires.ftc.teamcode.teleOps.debug.Test.kP;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class Launch extends CommandBase {
   private final IntakeSubsystem intake;
   private final TurretSubsystem turret;
   
   VoltageSensor battery;
   
   //schimbat valori din test
   PIDFController controller = new PIDFController(kP, kI, kD, kF);
   
   private TelemetryManager telemetryM;
   
   public Launch(TurretSubsystem turret, IntakeSubsystem intake) {
      this.intake = intake;
      this.turret = turret;
   }
   
   @Override
   public void initialize() {
      telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
      
      battery = hardwareMap.voltageSensor.iterator().next();
   }
   
   @Override
   public void execute() {
      turret.setSpeed(1200);
      turret.setAngle(0.04);
      controller.setPIDF(kP, kI, kD, kF * 12.0 / battery.getVoltage());
      turret.setPower(controller.calculate(turret.getVelocity(), turret.targetVelocity));
      if(turret.atSpeed())
         new Suck(intake);
      
      telemetryM.addData("viteza: ", turret.getVelocity());
   }
   
   @Override
   public void end(boolean interrupted) {
      turret.stop();
      intake.stop();
   }
}
