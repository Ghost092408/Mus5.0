package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig driveConfig = new MecanumConfig(
       c -> {
           c.frontLeftName.set("stangaFata");
           c.backLeftName.set("stangaSpate");
           c.frontRightName.set("dreaptaFata");
           c.backRightName.set("dreaptaSpate");
           
           c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
           c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
           c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
           c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
       }
    );
   
   public static PinpointConfig localizerConfig = new PinpointConfig(
      c -> {
         c.name.set("pinpoint");
         c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
         c.xPodOffset.set(52.82);
         c.yPodOffset.set(54.55);
         c.offsetUnits.set(DistanceUnit.MM);
         c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
         c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
      }
   );
   
   public static ForesightConfig foresightConfig = new ForesightConfig(
      c -> {
      
      }
   );
   
   public static Follower create(HardwareMap h) {
      return new Follower(
         new PinpointLocalizer(h, localizerConfig),
         new Mecanum(h, driveConfig),
         new Foresight(foresightConfig)
      );
   }
}