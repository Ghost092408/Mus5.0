package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.pedro.Drawing.ROBOT_RADIUS;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.LocalisationSubsystem;

public class TurnToHive extends CommandBase {
   private final LocalisationSubsystem localisation;
   Command turnCommand = null;
   
   Follower follower;
   
   private TelemetryManager telemetryM;
   
   public TurnToHive (LocalisationSubsystem localisation){
      this.localisation = localisation;
   }
   
   //coord hive1
   public static double hive1x = 60 - ROBOT_RADIUS;
   public static double hive1y = 59 - ROBOT_RADIUS;
   //coord hive2
   public static double hive2x = 84 - ROBOT_RADIUS;
   public static double hive2y = 59 - ROBOT_RADIUS;
   
   @Override
   public void initialize() {
      telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
   }
   
   @Override
   public void execute() {
      Pose pose = follower.pose();
      
      double x, y;
      if(pose.x() < 72) {
         x = hive1x;
         y = hive1y;
      }else{
         x = hive2x;
         y = hive2y;
      }
      
      turnCommand = new TurnToCommand(follower, localisation.getAngle(x, y), AngleUnit.DEGREES);
      CommandScheduler.getInstance().schedule(turnCommand);
   }
   
   @Override
   public void end (boolean interrupted) {
      turnCommand.cancel();
   }
}
