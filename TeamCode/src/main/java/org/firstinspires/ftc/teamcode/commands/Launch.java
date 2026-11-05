package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class Launch extends CommandBase {
   private final TurretSubsystem turret;
   
   private TelemetryManager telemetryM;
   
   public Launch(TurretSubsystem turret) { this.turret = turret; }
   
   @Override
   public void initialize() { telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); }
   
   @Override
   public void execute() {
      turret.setSpeed(2000);
      turret.update();
   }
   
   @Override
   public void end(boolean interrupted) {
      turret.setSpeed(0);
      turret.update();
   }
}
