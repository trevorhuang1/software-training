package frc.robot.commands.superstructure;

import frc.robot.subsystems.arm.ArmIO.ArmData;

public interface SuperStructureCommandInterface {

    /** Updates the set of loggable inputs. */
    public default void execute() {

    };

    public default void autoExecute() {

    };

    public default void reset() {

    };

    public default void autoReset(){

    }

    public default void start() {

    }
    public default void autoStart(){
        
    }
}
