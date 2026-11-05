package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.commands.Suck;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import kotlin.ConsistentCopyVisibility;

@TeleOp(name = "TeleOp")
@Configurable
public class TeleOp extends OpMode {
   private TelemetryManager telemetryM;
   
   private Follower follower;
   
   private GamepadEx gamepad;
   
   private IntakeSubsystem intake;
   
   @Override
   public void init() {
      telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
      
      follower = Constants.create(hardwareMap);
      follower.update();
      
      gamepad = new GamepadEx(gamepad1);
      
      //subsystems
      {
         intake = new IntakeSubsystem(hardwareMap);
      }
      
      new GamepadButton(gamepad, GamepadKeys.Button.CROSS)
         .toggleWhenPressed(new Suck(intake));
   }
   
   @Override
   public void loop(){
      follower.update();
      telemetryM.update();
      
      CommandScheduler.getInstance().run();
      
      follower.manual(
         -gamepad1.left_stick_y,
         gamepad1.left_stick_x,
         gamepad1.right_stick_x
      );
      
      Pose pose = follower.pose();
      
      telemetryM.debug("x: ", pose.x());
      telemetryM.debug("y: ", pose.y());
      telemetryM.debug("heading: ", Math.toDegrees(pose.heading()));
      telemetryM.update(telemetry);
   }
}
