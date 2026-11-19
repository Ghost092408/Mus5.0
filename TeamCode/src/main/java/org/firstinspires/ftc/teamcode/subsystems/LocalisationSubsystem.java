package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedro.Drawing.ROBOT_RADIUS;
import static java.lang.Math.abs;
import static java.lang.Math.atan;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class LocalisationSubsystem extends SubsystemBase {
   Follower follower;
   
   public LocalisationSubsystem(HardwareMap hardwareMap) {
      follower = Constants.create(hardwareMap);
      
      follower.update();
   }
   
   public double getDist(double x2, double y2){
      Pose pose = follower.pose();
      
      double x1 = pose.x();
      double y1 = pose.y();
      
      return Math.sqrt((x2-x1) * (x2-x1) + (y2-y1) * (y2-y1));
   }
   
   public double getAngle (double x2, double y2) {
      Pose pose = follower.pose();
      
      double x1 = pose.x();
      double y1 = pose.y();
      
      x2 -= ROBOT_RADIUS;
      y2 -= ROBOT_RADIUS;
      
      return Math.toDegrees(Math.atan2(y2 - y1, x2 - x1));
   }
}