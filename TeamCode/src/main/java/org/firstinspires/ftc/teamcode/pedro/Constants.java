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
   public static MecanumConfig driveConfig = new MecanumConfig(c -> {
      c.frontLeftName.set("stangaFata");
      c.frontRightName.set("dreaptaFata");
      c.backLeftName.set("stangaSpate");
      c.backRightName.set("dreaptaSpate");
      c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
      c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
      c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
      c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
   });
   
   public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
      c.name.set("pinpoint");
      c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
      c.xPodOffset.set(144.829);
      c.yPodOffset.set(54.552);
      c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
      c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
      c.globalDistanceUnit.set(DistanceUnit.INCH);
      c.offsetUnits.set(DistanceUnit.MM);
   });
   
   public static ForesightConfig foresightConfig = new ForesightConfig(
      c -> {
         Controller primaryTranslationalForward = Controller.proportional(0.5400896246475573);
         Controller secondaryTranslationalForward = Controller.proportional(0.19954868484671384);
         Controller primaryTranslationalLateral = Controller.proportional(1.3505842741772394);
         Controller secondaryTranslationalLateral = Controller.proportional(0.4990048010320366);
         
         c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
         c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
         
         c.coast.set(Controller.proportionalFeedforward(0.012387517710641586));
         c.brake.set(Controller.proportionalFeedforward(0.010529390054045348));
         
         c.headingFeedback.set(Controller.proportional(8.64615576170039));
         c.headingBrakeCoefficients.set(Vector2D.cartesian(0.063279122411118, 0.006236888469913614));
         
         c.linearBrakeCoefficients.set(Matrix.diag(0.13330961282917655, 0.055514834273407976));
         c.quadraticBrakeCoefficients.set(Matrix.diag(8.11218331993522E-4, 0.0019020215569548464));
         
         c.maxAchievableForwardVelocity.set(80.5043187802661);
         c.maxAchievableStrafeVelocity.set(62.016999272753196);
         c.naturalForwardDeceleration.set(46.18090410283323);
         c.naturalStrafeDeceleration.set(103.0758494337582);
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