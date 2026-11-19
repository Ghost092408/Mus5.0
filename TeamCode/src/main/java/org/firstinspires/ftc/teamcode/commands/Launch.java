package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.teleOps.debug.Test.kD;
import static org.firstinspires.ftc.teamcode.teleOps.debug.Test.kF;
import static org.firstinspires.ftc.teamcode.teleOps.debug.Test.kI;
import static org.firstinspires.ftc.teamcode.teleOps.debug.Test.kP;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalisationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class Launch extends CommandBase {
   private final IntakeSubsystem intake;
   private final TurretSubsystem turret;
   private final LocalisationSubsystem localisation;
   
   Follower follower;
   
   VoltageSensor battery;
   
   //schimbat valori din test
   PIDFController controller = new PIDFController(kP, kI, kD, kF);
   
   private TelemetryManager telemetryM;
   
   public Launch(TurretSubsystem turret, IntakeSubsystem intake, LocalisationSubsystem localisation) {
      this.intake = intake;
      this.turret = turret;
      this.localisation = localisation;
   }
   
   @Override
   public void initialize() {
      telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
      
      battery = hardwareMap.voltageSensor.iterator().next();
   }
   
   @Override
   public void execute() {
      Pose pose = follower.pose();
      
      double
         x = pose.x(),
         y = pose.y();
      
      double
         dist = localisation.getDist(x, y),
         dist2 = dist * dist,
         dist3 = dist2 * dist,
         dist4 = dist3 * dist;
      double rampPos = -0.00003892155 * dist4 + 0.007419355 * dist3 - 0.5288573 * dist2 + 16.71469 * dist - 197.50988;
      
      new TurnToHive(localisation);
      turret.setSpeed(1000);
      turret.setAngle(rampPos);
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
