package frc.robot.lights;

public enum Color {
    Black(0.99),
    Red(0.61),
    DarkBlue(0.85),
    RainbowWithRainbowPallette(-0.99);

    double power;

    Color(double power) {
        this.power = power;
    }


}
