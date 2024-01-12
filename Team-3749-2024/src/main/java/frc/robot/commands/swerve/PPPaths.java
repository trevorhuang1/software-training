package frc.robot.commands.swerve;

import java.util.HashMap;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;

public class PPPaths {
  public static void init_pathCommands(HashMap<String, Command> commandList) {
    commandList.forEach((String cmdName, Command cmd) -> {
      NamedCommands.registerCommand(cmdName, cmd);
    });
  }
}
