package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedro.Drawing.ROBOT_RADIUS;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class LocalisationSubsystem extends SubsystemBase {
   private Follower follower;
   
   public LocalisationSubsystem(HardwareMap hardwareMap) {
      this.follower = Constants.create(hardwareMap);
      this.follower.update();
   }

   public LocalisationSubsystem(Follower follower) {
      this.follower = follower;
   }

   public LocalisationSubsystem(HardwareMap hardwareMap, Follower follower) {
      this.follower = (follower != null) ? follower : Constants.create(hardwareMap);
   }

   public void setFollower(Follower follower) {
      this.follower = follower;
   }
   
   public double getDist(double x2, double y2){
      if (follower == null) return 0;
      Pose pose = follower.pose();
      
      double x1 = pose.x();
      double y1 = pose.y();
      
      return Math.sqrt((x2-x1) * (x2-x1) + (y2-y1) * (y2-y1));
   }
   
   public double getAngle (double x2, double y2) {
      if (follower == null) return 0;
      Pose pose = follower.pose();
      
      double x1 = pose.x();
      double y1 = pose.y();
      
      x2 -= ROBOT_RADIUS;
      y2 -= ROBOT_RADIUS;
      
      return Math.toDegrees(Math.atan2(y2 - y1, x2 - x1));
   }

   public Pose getPose() {
      if (follower != null) {
         return follower.pose();
      }
      return new Pose(0, 0, 0);
   }

   public Follower getFollower() {
      return follower;
   }
}
