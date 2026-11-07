package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class Launch extends CommandBase {
   private final IntakeSubsystem intake;
   private final TurretSubsystem turret;
   
   private TelemetryManager telemetryM;
   
   public Launch(TurretSubsystem turret, IntakeSubsystem intake) {
      this.intake = intake;
      this.turret = turret;
   }
   
   @Override
   public void initialize() { telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); }
   
   @Override
   public void execute() {
      turret.setSpeed(2000);
      turret.update();
      if(turret.atSpeed())
         intake.suge();
      
      telemetryM.addData("viteza: ", turret.getVelocity());
   }
   
   @Override
   public void end(boolean interrupted) {
      turret.stop();
      intake.stop();
   }
}
