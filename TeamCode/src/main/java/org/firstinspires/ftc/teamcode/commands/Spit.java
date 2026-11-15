package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class Spit extends CommandBase {
   private final IntakeSubsystem intake;
   
   private TelemetryManager telemetryM;
   
   public Spit(IntakeSubsystem intake) { this.intake = intake; }
   
   @Override
   public void initialize() { telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); }
   
   @Override
   public void execute() { intake.scuipa(); }
   
   @Override
   public void end(boolean interrupted) { intake.stop(); }
}
