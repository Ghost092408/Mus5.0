package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.LocalisationSubsystem;

public class TurnToHive extends CommandBase {
   private final LocalisationSubsystem localisation;
   private final Follower follower;
   private TurnToCommand turnCommand;
   
   private TelemetryManager telemetryM;
   
   public TurnToHive (Follower follower, LocalisationSubsystem localisation){
      this.follower = follower;
      this.localisation = localisation;
   }
   
   //coord hive1
   public static double hive1x = 60;
   public static double hive1y = 59;
   //coord hive2
   public static double hive2x = 84;
   public static double hive2y = 59;
   
   @Override
   public void initialize() {
      telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
      
      Pose pose = follower.pose();
      
      double x = (pose.x() < 72) ? hive1x : hive2x;
      double y = (pose.x() < 72) ? hive1y : hive2y;
      
      double targetAngle = localisation.getAngle(x, y);
      
      turnCommand = new TurnToCommand(follower, targetAngle, AngleUnit.DEGREES);
      turnCommand.initialize();
   }
   
   @Override
   public void execute() {
      if (turnCommand != null) {
         turnCommand.execute();
      }
   }
   
   @Override
   public void end (boolean interrupted) {
      if (turnCommand != null) {
         turnCommand.end(interrupted);
      }
   }
}
