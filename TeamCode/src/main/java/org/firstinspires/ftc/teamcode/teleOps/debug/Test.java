package org.firstinspires.ftc.teamcode.teleOps.debug;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.List;

@TeleOp
@Configurable
public class Test extends LinearOpMode {
   public void runOpMode() throws InterruptedException {
      DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "stangaFata");
      DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "dreaptaFata");
      DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "stangaSpate");
      DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "dreaptaSpate");
      DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
      DcMotorEx transfer = hardwareMap.get(DcMotorEx.class, "transfer");
      DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
      
//      GoBildaPinpointDriver odom = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
      
      frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
      backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
      backRight.setDirection(DcMotorSimple.Direction.REVERSE);
      intake.setDirection(DcMotorSimple.Direction.REVERSE);
      
      boolean intakeState = false;
      boolean intakeStateR = false;
      boolean transferState = false;
      boolean transferStateR = false;
      boolean turretState = false;
      boolean drivingState = true;
      
      double turretPow = 1, intakePow = 1;
      
      Gamepad currentGamepad1 = new Gamepad();
      Gamepad currentGamepad2 = new Gamepad();
      
      Gamepad previousGamepad1 = new Gamepad();
      Gamepad previousGamepad2 = new Gamepad();
      
      List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
      
      for (LynxModule hub : allHubs) {
         hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
      }
      
      //odometrie
//      odom.setOffsets(-84.0, -168.0, DistanceUnit.MM);
//      odom.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//      odom.setEncoderDirections(
//         GoBildaPinpointDriver.EncoderDirection.FORWARD,
//         GoBildaPinpointDriver.EncoderDirection.FORWARD
//      );
//      odom.resetPosAndIMU();
      
      waitForStart();
      
      while(opModeIsActive() && !isStopRequested()) {
         for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
         }
         
         //odometrie
//         odom.update();
//         Pose2D pos = odom.getPosition();
         
         // Store gamepad values from the previous loop iteration
         previousGamepad1.copy(currentGamepad1);
         previousGamepad2.copy(currentGamepad2);
         
         // Store gamepad values from this loop iteration
         currentGamepad1.copy(gamepad1);
         currentGamepad2.copy(gamepad2);
         
         //sistem de miscare
         double y = -gamepad1.left_stick_y;
         double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
         double rx = gamepad1.right_stick_x;
         
         double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
         double frontLeftPower = (y + x + rx) / denominator;
         double backLeftPower = (y - x + rx) / denominator;
         double frontRightPower = (y - x - rx) / denominator;
         double backRightPower = (y + x - rx) / denominator;
         
         if(previousGamepad1.touchpad && !currentGamepad1.touchpad)
            drivingState = !drivingState;
         if (drivingState) {
            //miscare
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
         }
         
         //intake
         if (previousGamepad1.a && !currentGamepad1.a) {
            intakeState = !intakeState;
         }
         if (intakeState && !intakeStateR) {
            intake.setPower(intakePow);
         } else {
            intake.setPower(0);
         }
         
         if (previousGamepad1.b && !currentGamepad1.b) {
            intakeStateR = !intakeStateR;
         }
         if (intakeStateR && !intakeState) {
            intake.setPower(-intakePow);
         } else if(!intakeStateR && !intakeState) {
            intake.setPower(0);
         }
         
         //transfer
         if (previousGamepad1.x && !currentGamepad1.x) {
            transferState = !transferState;
         }
         if (transferState && !transferStateR) {
            transfer.setPower(intakePow);
         } else {
            transfer.setPower(0);
         }
         
         if (previousGamepad1.y && !currentGamepad1.y) {
            transferStateR = !transferStateR;
         }
         if (transferStateR && !transferState) {
            transfer.setPower(-intakePow);
         } else if(!transferStateR && !transferState) {
            transfer.setPower(0);
         }
         
         //turret
         if (previousGamepad1.right_bumper && !currentGamepad1.right_bumper) {
            turretState = !turretState;
         }
         if (turretState) {
            turret.setPower(turretPow);
         } else {
            turret.setPower(0);
         }
         
         //intake power
         if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up)
            if(intakePow < 1) intakePow += 0.1;
         if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down)
            if(intakePow > 0) intakePow -= 0.1;
         
         //turret power
         if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right)
            if(turretPow < 1) turretPow += 0.1;
         if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left)
            if(turretPow > 0) turretPow -= 0.1;
         
         //telemetry
//         telemetry.addData("x", pos.getX(DistanceUnit.INCH));
//         telemetry.addData("y", pos.getY(DistanceUnit.INCH));
//         telemetry.addData("heading", pos.getHeading(AngleUnit.DEGREES));
         telemetry.addData("power: ", intakePow);
         telemetry.update();
      }
   }
}
