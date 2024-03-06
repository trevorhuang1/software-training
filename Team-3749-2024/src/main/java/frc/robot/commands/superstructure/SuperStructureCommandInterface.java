package frc.robot.commands.superstructure;

import frc.robot.subsystems.arm.ArmIO.ArmData;

public interface SuperStructureCommandInterface {

    /** Updates the set of loggable inputs. */
    public default void execute() {

    };
    public default void execute(double goalRad) {

    };
    public default void reset(){

    };
    
}
