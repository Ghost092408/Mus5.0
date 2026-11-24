package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.teleOps.debug.Test.kD;
import static org.firstinspires.ftc.teamcode.teleOps.debug.Test.kF;
import static org.firstinspires.ftc.teamcode.teleOps.debug.Test.kI;
import static org.firstinspires.ftc.teamcode.teleOps.debug.Test.kP;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalisationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class Launch extends CommandBase {
   private final IntakeSubsystem intake;
   private final TurretSubsystem turret;
   private final LocalisationSubsystem localisation;
   private final HardwareMap hardwareMap;
   
   private TurnToCommand turnCommand;
   private VoltageSensor battery;
   private final ElapsedTime rampTimer = new ElapsedTime();
   private boolean hasReachedSpeed = false;

   // schimbat valori din test
   private final PIDFController controller = new PIDFController(kP, kI, kD, kF);
   
   private TelemetryManager telemetryM;
   
   public Launch(HardwareMap hardwareMap, TurretSubsystem turret, IntakeSubsystem intake, LocalisationSubsystem localisation) {
      this.hardwareMap = hardwareMap;
      this.turret = turret;
      this.intake = intake;
      this.localisation = localisation;
   }

   public Launch(TurretSubsystem turret, IntakeSubsystem intake, LocalisationSubsystem localisation) {
      this(null, turret, intake, localisation);
   }
   
   @Override
   public void initialize() {
      try {
         telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
      } catch (Exception ignored) {}
      
      if (hardwareMap != null && hardwareMap.voltageSensor != null && hardwareMap.voltageSensor.iterator().hasNext()) {
         battery = hardwareMap.voltageSensor.iterator().next();
      }

      rampTimer.reset();
      hasReachedSpeed = false;

      if (localisation != null && localisation.getFollower() != null) {
         Follower follower = localisation.getFollower();
         Pose pose = localisation.getPose();
         double targetX = (pose.x() < 72) ? 60.0 : 84.0;
         double targetY = 59.0;
         double targetAngle = localisation.getAngle(targetX, targetY);

         turnCommand = new TurnToCommand(follower, targetAngle, AngleUnit.DEGREES);
         turnCommand.initialize();
      }
   }
   
   @Override
   public void execute() {
      if (turnCommand != null) {
         turnCommand.execute();
      }

      Pose pose = (localisation != null) ? localisation.getPose() : new Pose(0, 0, 0);
      
      double targetX = (pose.x() < 72) ? 60.0 : 84.0;
      double targetY = 59.0;
      
      double dist = (localisation != null) ? localisation.getDist(targetX, targetY) : 0;
      double dist2 = dist * dist;
      double dist3 = dist2 * dist;
      double dist4 = dist3 * dist;
      
      double rampPos = -0.00003892155 * dist4 + 0.007419355 * dist3 - 0.5288573 * dist2 + 16.71469 * dist - 197.50988;
      
      if (turret != null) {
         turret.setSpeed(1000);
         turret.setAngle(rampPos);
         double voltageComp = (battery != null) ? (12.0 / battery.getVoltage()) : 1.0;
         controller.setPIDF(kP, kI, kD, kF * voltageComp);
         turret.setPower(controller.calculate(turret.getVelocity(), turret.targetVelocity));

         if (turret.atSpeed() && rampTimer.seconds() > 0.4) {
            hasReachedSpeed = true;
         }

         if (hasReachedSpeed && intake != null) {
            intake.setPowInt(1);
            intake.setPowTrans(1);
         } else if (intake != null) {
            intake.setPowTrans(0);
         }
      }
      
      if (telemetryM != null) {
         telemetryM.addData("viteza: ", (turret != null) ? turret.getVelocity() : 0);
      }
   }
   
   @Override
   public void end(boolean interrupted) {
      if (turnCommand != null) {
         turnCommand.end(interrupted);
      }
      if (turret != null) turret.stop();
      if (intake != null) intake.stop();
   }
}
