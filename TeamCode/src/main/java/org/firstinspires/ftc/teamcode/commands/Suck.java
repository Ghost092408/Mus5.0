package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class Suck extends CommandBase {
   private final IntakeSubsystem intake;
   
   private TelemetryManager telemetryM;
   
   public Suck(IntakeSubsystem intake) { this.intake = intake; }
   
   @Override
   public void initialize() { telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); }
   
   @Override
   public void execute() { intake.suge(); }
   
   @Override
   public void end(boolean interrupted) { intake.store(); }
}
