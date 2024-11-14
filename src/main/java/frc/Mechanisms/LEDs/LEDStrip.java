package frc.Mechanisms.LEDs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubmoduleSubsystemConstants.ConstLEDs;

public class LEDStrip extends SubsystemBase {

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue = 0;
    private int m_startOrangeLed = 0;
    private int LEDLen;

    int LedId, ODSReading;
    String LedMode;
    private ShuffleboardTab sbTab;
    private GenericEntry sbLedPort, sbLedMode;


    public LEDStrip(int LEDPort, int LEDLength) {
        LEDLen = LEDLength;
        LedId = LEDPort;
        LedMode = "None";
        // Must be a PWM header, not MXP or DIO. WIRING: PWM RED AND GREEN MUST BE SWITCHED FOR LEDS TO WORK
        m_led = new AddressableLED(LEDPort);
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(LEDLength);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

        dashboardInit();
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        dashboardUpdate();
  }

    public void setLedBufferByGroup(int start, int stop, ConstLEDs.COLORS color){
        // check to see if stop point exceeds the length of the LED sting
        if (stop >= m_ledBuffer.getLength()) {
            stop = m_ledBuffer.getLength() - 1;
        }
        // interate through start and stop 
        for (int i = start; i <= stop; i++) {
            switch(color){
                case GREEN:
                    m_ledBuffer.setRGB(i, 0, 255, 0);
                    break;
                case RED:
                    m_ledBuffer.setRGB(i, 255, 0, 0);
                    break;
                case REDDISHORANGE:
                    m_ledBuffer.setRGB(i, 255, 20, 0);
                    break;
                case PURPLE:
                    m_ledBuffer.setRGB(i, 255, 0, 255);
                    break;
                case YELLOW:
                    m_ledBuffer.setRGB(i, 255, 100, 0);
                    break;
                case ORANGE:
                    m_ledBuffer.setRGB(i, 255, 50, 0);
                    break;
                case WHITE:
                    m_ledBuffer.setRGB(i, 255, 255, 255);
                    break;
                case OFF:
                    m_ledBuffer.setRGB(i, 0, 0, 0);
                    break;
            }
            
        }
    }

    public void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }

    public void orangeWave(){
        for (var i = 0; i < m_ledBuffer.getLength(); i++){
            if ((i % 12) >= m_startOrangeLed && (i % 12) < m_startOrangeLed +4) {
                setLedBufferByGroup(i, i, ConstLEDs.COLORS.ORANGE);
            }
            else if ((i % 12) >= m_startOrangeLed +4 && (i % 12) < m_startOrangeLed +8) {
                setLedBufferByGroup(i, i, ConstLEDs.COLORS.YELLOW);
            }
            else {
                setLedBufferByGroup(i, i, ConstLEDs.COLORS.RED);
            }
        }
        setLeds();
        // m_startOrangeLed+=1;
        // if (m_startOrangeLed>m_ledBuffer.getLength()){
        //     m_startOrangeLed=0;
        // }
    }

    public void setLedsOff() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 0, 0, 0);
         }
    }

    public void setLeds(){
        // Set the LEDs
        m_led.setData(m_ledBuffer);
    }

    public int getLedLength(){
        return LEDLen;
    }

    public void UpdateLedMode(String Mode){
        LedMode = Mode;
    }

    private void dashboardInit() {
        sbTab = Shuffleboard.getTab("LED");
        sbLedPort = sbTab.add("LED Port", LedId).getEntry();
        sbLedMode = sbTab.add("LED Mode", LedMode).getEntry();
    }

    public void dashboardUpdate(){
        sbLedPort.setInteger(LedId);
        sbLedMode.setString(LedMode);
    }
}
