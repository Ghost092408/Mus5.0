package org.firstinspires.ftc.teamcode.teleOps.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.commands.Launch;
import org.firstinspires.ftc.teamcode.commands.Spit;
import org.firstinspires.ftc.teamcode.commands.Suck;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp (name = "TeleOpOf")
@Configurable
public class TeleOpOf extends OpMode {
   private TelemetryManager telemetryM;
   
   private Follower follower;
   
   @Override
   public void init() {
      telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
      
      follower = Constants.create(hardwareMap);
      follower.setPose(new Pose(0, 0, 0));
      follower.update();
      
      GamepadEx gamepad = new GamepadEx(gamepad1);
      
      //subsysteme
      TurretSubsystem turret;
      IntakeSubsystem intake;
      
      {
         intake = new IntakeSubsystem(hardwareMap);
         turret = new TurretSubsystem(hardwareMap);
      }
      
      //buttoane
     new GamepadButton(gamepad, GamepadKeys.Button.CROSS)
        .toggleWhenPressed( new Suck(intake) );
      new GamepadButton(gamepad, GamepadKeys.Button.CIRCLE)
         .toggleWhenPressed( new Spit(intake) );
     new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP)
        .toggleWhenPressed( new Launch(turret, intake) );
   }
   
   @Override
   public void loop() {
      follower.update();
      telemetryM.update();
      
      Pose pose = follower.pose();
      
      CommandScheduler.getInstance().run();
      
      //miscare - field centric
      DrivePowers powers = ManualDrive.fieldCentric(
         -gamepad1.left_stick_y,
         -gamepad1.left_stick_x,
         -gamepad1.right_stick_x,
         follower.pose().heading()
      );
      follower.manual(powers);
      
      //telemetrie
      telemetryM.debug("x: ", pose.x());
      telemetryM.debug("y: ", pose.y());
      telemetryM.debug("heading: ", Math.toDegrees(pose.heading()));
      telemetryM.update(telemetry);
   }
}
