package frc.robot.Commands.SensorAndLEDCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstLEDs.COLORS;
import frc.robot.Subsystems.LEDs;


public class LEDsWhite extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final LEDs Leds;
    /**
     * @param subsystem The subsystem used by this command.
     */
    public LEDsWhite(LEDs subsystem) {
      Leds = subsystem;
      addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute(){
      Leds.setLedBufferByGroup(0, Leds.getLedLength(), COLORS.WHITE);  
      Leds.setLeds();
      Leds.UpdateLedMode("White");
  }
}
