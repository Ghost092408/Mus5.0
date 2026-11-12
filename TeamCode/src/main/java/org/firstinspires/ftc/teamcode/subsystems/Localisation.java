package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.tan;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Localisation extends SubsystemBase {
   Follower follower;
   Pose pose = follower.pose();
   
   public Localisation (HardwareMap hardwareMap) {
      follower = Constants.create(hardwareMap);
      
      follower.update();
   }
   
   public void getAngle (double x2, double y2) {
      double x1 = pose.x();
      double y1 = pose.y();
      
      double c1 = abs(x2 - x1);
      double c2 = abs(y2 - y1);
      
      double angle = -1;
      
      if(x2 > x1) {
         if(y2 > y1){
            angle = Math.toDegrees(atan(c1/c2));
         }
      }else {
      
      }
      
//      return angle;
   }
   
}