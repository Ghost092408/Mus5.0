package org.firstinspires.ftc.teamcode.teleOps.debug;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@TeleOp
@Configurable
public class Test extends LinearOpMode {
   public static double turretSpeed = 0;
   public static double rampPos = 0;
   
   public static double kP = 0.0048;
   public static double kI = 0;
   public static double kD = 0;
   public static double kF = 0.000595;
   
   public void runOpMode() throws InterruptedException {
      TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
      
      DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "stangaFata");
      DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "dreaptaFata");
      DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "stangaSpate");
      DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "dreaptaSpate");
      DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
      DcMotorEx transfer = hardwareMap.get(DcMotorEx.class, "transfer");
      DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
      
      VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
      
//      ServoImplEx rampaAng = hardwareMap.get(ServoImplEx.class, "rampAng");
      
      Follower follower;;
      
      follower = Constants.create(hardwareMap);
      follower.update();
      
      PIDFController controller = new PIDFController(kP, kI, kD, kF);
      
      frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
      backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
      backRight.setDirection(DcMotorSimple.Direction.REVERSE);
      
      turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      
      boolean intakeState = false;
      boolean intakeStateR = false;
      boolean transferState = false;
      boolean transferStateR = false;
      boolean turretState = false;
      boolean drivingState = true;
      
      Gamepad currentGamepad1 = new Gamepad();
      Gamepad currentGamepad2 = new Gamepad();
      
      Gamepad previousGamepad1 = new Gamepad();
      Gamepad previousGamepad2 = new Gamepad();
      
      List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
      
      for (LynxModule hub : allHubs) {
         hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
      }
      
      waitForStart();
      
      while(opModeIsActive() && !isStopRequested()) {
         for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
         }
         
         //odometrie
         follower.update();
         Pose pose = follower.pose();
         
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
         
         //miscare
         if(previousGamepad1.touchpad && !currentGamepad1.touchpad)
            drivingState = !drivingState;
         if (drivingState) {
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
         }
         
//         rampaAng.setPosition(rampPos);
         
         //intake
         if (previousGamepad1.a && !currentGamepad1.a) {
            intakeStateR = false;
            intakeState = !intakeState;
         }
         if(intakeState)
            intake.setPower(1);
         else if(!intakeStateR)
            intake.setPower(0);
         
         if (previousGamepad1.b && !currentGamepad1.b) {
            intakeState = false;
            intakeStateR = !intakeStateR;
         }
         if(intakeStateR)
            intake.setPower(-1);
         else if(!intakeState)
            intake.setPower(0);
         
         //transfer
         if (previousGamepad1.x && !currentGamepad1.x) {
            transferStateR = false;
            transferState = !transferState;
         }
         if(transferState)
            transfer.setPower(1);
         else if(!transferStateR)
            transfer.setPower(0);
         
         if (previousGamepad1.y && !currentGamepad1.y) {
            transferState = false;
            transferStateR = !transferStateR;
         }
         if(transferStateR)
            transfer.setPower(-1);
         else if(!transferState)
            transfer.setPower(0);
         
         //turret PIDF
         if (previousGamepad1.right_bumper && !currentGamepad1.right_bumper) {
            turretState = !turretState;
            controller.reset();
         }
         if (turretState) {
            controller.setPIDF(kP, kI, kD, kF * 12.0 / battery.getVoltage());
            turret.setPower(controller.calculate(turret.getVelocity(), turretSpeed));
         }
         else
            turret.setPower(0);
         
         //ramp position
         if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up)
            if(rampPos < 1) rampPos += 0.01;
         if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down)
            if(rampPos > 0) rampPos -= 0.01;
         
         //telemetry
         panelsTelemetry.addData("x: ", pose.x());
         panelsTelemetry.addData("y: ", pose.y());
         panelsTelemetry.addData("heading: ", Math.toDegrees(pose.heading()));
         panelsTelemetry.addData("turret state: ", turretState);
         panelsTelemetry.addData("ramp angle", rampPos);
         panelsTelemetry.addData("viteza tureta: ", turret.getVelocity());
         panelsTelemetry.addData("kF ", kF);
         panelsTelemetry.addData("kP ", kP);
         panelsTelemetry.addData("kI ", kI);
         panelsTelemetry.addData("kD ", kD);
         panelsTelemetry.update(telemetry);
      }
   }
}
