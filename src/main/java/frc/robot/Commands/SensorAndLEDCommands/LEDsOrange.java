package frc.robot.Commands.SensorAndLEDCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LEDs;


public class LEDsOrange extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final LEDs Leds;
    /**
     * @param subsystem The subsystem used by this command.
     */
    public LEDsOrange(LEDs subsystem) {
      Leds = subsystem;
      addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute(){
      Leds.orangeWave();
      Leds.setLeds();
      Leds.UpdateLedMode("Orange Wave");
  }
}
