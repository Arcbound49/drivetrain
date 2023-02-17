package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Lights {
    public int LED_Length = 23;
    public double time = 0;

    public AddressableLED LED = new AddressableLED(0);
    public AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LED_Length);
    public void Lights() {
        while (true) {
            time += .005;
            for (int index = 0; index < ledBuffer.getLength(); index++) {

                final double constant = index / (ledBuffer.getLength() * (Math.PI/2));

                double green = Math.sin(time + constant);
                double blue = Math.cos(time + constant);
                double red = -Math.sin(time + constant);

                green *= 255/2;
                blue *= 255/2;
                red *= 255/2;

                green += 255/2;
                blue += 255/2;
                red += 255/2;

              ledBuffer.setRGB(index, (int) red, (int) green, (int) blue);
            }
        }
    }
}