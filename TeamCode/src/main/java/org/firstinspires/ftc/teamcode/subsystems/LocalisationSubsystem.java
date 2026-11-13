package org.firstinspires.ftc.teamcode.subsystems;

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
   private static final double FIELD_CENTER = 7.87;
   Follower follower;
   
   public LocalisationSubsystem(HardwareMap hardwareMap) {
      follower = Constants.create(hardwareMap);
      
      follower.update();
   }
   
   public double dist(double x2, double y2){
      Pose pose = follower.pose();
      
      double x1 = pose.x();
      double y1 = pose.y();
      
      return Math.sqrt((x2-x1) * (x2-x1) + (y2-y1) * (y2-y1));
   }
   
   public double getAngle (double x2, double y2) {
      Pose pose = follower.pose();
      
      double x1 = pose.x();
      double y1 = pose.y();
      
      x2 += FIELD_CENTER;
      y2 += FIELD_CENTER;
      
      double c1 = abs(x2 - x1);
      double c2 = abs(y2 - y1);
      
      double angle = -1;
      double degrees = Math.toDegrees(atan(c2/c1));
      
      
      if(x2 > x1) {
         if(y2 > y1)
            angle = degrees;
         else
            angle = 360 - degrees;
      }else {
         if(y2 > y1)
            angle = 180 - degrees;
         else
            angle = 180 + degrees;
      }
      
      return angle;
   }
   
   public void turnTo(double angle) {
      new TurnToCommand(follower, angle, AngleUnit.DEGREES);
   }
}