package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;

public record PIDConstants (double kP, double kI, double kD, double iZone, double kTolerance){
    public PIDConstants(double kP, double kI, double kD, double iZone) {
        this(kP, kI, kD, iZone, 0.0);
    }
    public PIDConstants(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.0, 0.0);
    }

    public PIDController getController() {
        PIDController controller = new PIDController(kP, kI, kD);
        controller.setIZone(iZone);
        controller.setTolerance(kTolerance);
        return controller;
    }

    public double getTolerance() {
        return kTolerance;
    }
}
/*
⠀⠀⠀⠀⠀⠀⠀⠀⢠⣾⣯⣟⡆⢸⢠⣯⡃⠀⠀⠀⠀⢹⣆⠀⠀⠀⠁⠀⠀⠀⠀⠀⠀⠀⢈⣿⠉⢹⢉⢹⡌⢉⡜⠚⠭⠤⠋⠑⣼⣿⢟⡿⣿⣻⣯⣸⡇⣿⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⣦⡄⠀⠀⠀⠀⢸⡿⢄⣈⠀⢸⣿⣿⠃⠀⠀⠀⠀⡎⣿⡄⠀⠀⠀⠀⠀⢀⣀⣤⣶⣾⣿⠟⠙⢸⠯⣈⠋⠉⠀⣼⣾⣦⣤⣶⣿⣶⡟⢼⣺⣝⠾⠕⠙⠻⣽⣖⣤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠘⡟⠙⢦⡀⠀⠖⠛⠛⠺⠶⣉⢦⣷⡇⠀⠀⠀⠀⠀⠸⡸⣷⡆⣀⣤⣴⣾⡿⠟⠛⠉⠀⠈⠛⠛⠻⢦⣼⣶⣶⣾⣿⣿⣿⣿⣿⣿⣿⣷⣖⣬⣴⢷⣶⡴⣶⣿⣦⣤⡙⠻⣶⣄⠀⠀⠀⠀⠀⠀⠀
⠀⠈⠀⠀⠀⠹⢦⡀⠀⠀⠀⠀⠀⠙⢿⣿⡄⠀⠀⠀⠀⠀⢣⢻⣿⡿⠖⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠙⠻⢿⣿⣿⣿⣿⣿⣿⣿⡏⣾⣿⣿⡜⢿⣿⣮⢻⣿⣿⣿⣷⣾⢻⡆⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠈⠙⢦⡀⠀⠀⠀⠀⠀⠙⢷⡀⠀⠀⢀⣠⣼⣿⡟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣤⣴⣶⠿⠟⠛⠋⠉⠁⠀⣼⢰⡿⢿⣿⣿⣮⣛⢿⡇⣿⣿⣿⢿⣷⡿⠁⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢦⡀⠀⠀⠀⠀⠀⣳⣰⣾⣿⣿⣿⣿⠃⠀⠀⠀⠀⠀⠀⠀⣀⣤⣴⣶⣿⣿⣿⣍⣄⣀⣀⣀⡐⣀⣠⣠⣄⣿⢸⣿⣿⣿⣿⣿⣯⡟⣰⣷⣿⣿⣿⠟⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢦⡀⢠⣴⣿⣿⣿⣿⣿⣿⣿⡏⠀⠀⠀⢀⣠⣴⣾⣿⣿⣿⣿⣿⠿⠿⠿⠿⢿⣿⡛⢟⠛⠛⠛⠛⠛⠿⣿⡹⣻⠷⢿⣛⣭⣾⠟⢿⣿⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠁⣠⣴⣾⣿⣿⣿⣿⡿⠟⠋⠁⠀⠀⠀⠀⠀⠀⠉⡻⣮⡳⣄⠀⠀⠀⠀⠨⡻⡻⡟⢻⣿⡿⣇⠀⠈⢿⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠁⠀⠀⠀⠀⠀⠀⠀⢀⣠⣾⠟⠙⠁⠙⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠟⠋⠁⠀⠀⠀⠀⠀⠀⢀⡠⠀⠤⠤⠐⠌⢿⣾⣦⡀⠀⠀⠀⢳⡙⢟⡄⠹⡿⡇⡆⠀⠈⣿⣷⣄⠀⠀⠀⠀⠀⠀⠀⠀
⣌⠢⣄⠀⠀⠀⠀⣰⣿⢿⠃⠀⠀⠀⠐⢿⣿⣿⣿⣿⠟⠋⣁⣾⣿⡟⠉⠀⠀⠀⠀⠀⢀⡈⠲⢜⡛⠥⠄⠖⠉⠁⢀⣀⣀⣘⢳⣵⡄⠀⠀⠀⢷⠘⣽⡄⢱⡇⢣⠀⠀⠘⣿⣿⣿⣦⡀⠀⠀⠀⠀⠀
⠈⠳⣌⠳⣄⣤⣾⠟⢡⠏⠀⠀⠀⠀⢀⣬⡿⠟⠉⠀⠀⣠⡿⠋⠘⢿⡦⣄⠀⢀⠤⠐⠁⠀⠀⠀⠹⢶⣤⣶⠾⠛⠉⠁⠀⢀⣰⠙⣿⡄⠀⠀⠘⡇⢸⢷⠘⣷⠀⠀⠀⠀⢹⣿⣿⣿⣿⣷⣄⡀⠀⠀
⠀⠀⠈⠳⣼⡽⠋⢀⡞⠀⠀⠀⣠⣴⠟⠉⢀⡄⠀⢀⣴⡋⠀⠀⠀⡄⠙⠮⣷⣤⡤⠜⠁⠀⠀⣠⡾⠟⠉⠛⠿⣿⣭⠿⠿⣿⣿⣿⣽⣷⡀⠀⠀⣽⠸⣛⣆⡇⠀⠀⠀⠀⠀⢿⣿⣧⡙⣿⣿⣿⣶⣤
⠀⠀⠀⠀⡿⠀⠀⣼⠁⢀⣤⡾⠋⠀⠀⠀⠘⣿⣤⠞⠀⠙⢦⣄⢸⠁⠀⠀⠈⠑⠿⣶⣤⣤⣾⠋⠀⠀⣀⣀⣀⠁⠈⠙⢒⣦⣤⣼⣥⣬⢷⡀⠀⢸⠀⣿⣿⡇⠀⠀⠀⠀⠀⠸⣷⠙⢿⣌⠹⠟⠙⠻
⠀⠀⠀⢰⠃⠀⣸⢁⣴⣿⠋⠀⠀⠀⠀⠀⠀⡾⣷⡀⣀⢀⡴⠙⢷⣍⠢⢄⠏⠉⠙⠮⠉⠻⢤⡛⠉⠉⠉⠁⢀⣬⣭⣿⣿⣿⣿⠿⢿⣿⡘⢧⠀⢸⣸⡏⣿⠁⠀⢠⠀⠀⠀⡇⢻⡤⠀⠙⣷⣄⠀⠀
⠀⠀⢀⡏⢀⣼⣵⣿⣿⣿⡀⠀⠀⠀⠀⠀⡼⠁⢈⢿⡆⠑⠦⣀⠀⣙⣻⢄⡀⠀⠀⠀⠀⠀⠀⠉⠳⢦⣀⣞⣵⣿⣿⣫⡯⡭⡿⢷⠀⣿⠃⠸⡇⠘⣿⢧⡏⠀⠀⠏⠀⣀⣀⢧⠸⣧⡀⠀⠀⠙⢷⡔
⠀⠀⣸⢡⣮⣾⣿⣿⣿⡇⣿⡄⠀⠀⠀⣸⠁⣠⠏⡼⣿⣶⣿⣂⣥⡤⡄⠀⠉⠛⢿⣵⣦⣀⣀⣀⡀⣀⣈⣙⣻⢿⣣⡮⡅⠈⠛⢽⣧⡏⠀⠀⣷⣼⡿⣾⡁⠀⠐⠀⠀⠀⠀⢈⣱⡇⠙⢷⣤⡀⠀⠉
⠀⢠⣧⣿⣿⣿⣿⣿⣿⣇⡇⢷⠀⠀⣸⢻⡂⠃⡰⠁⣧⣷⠟⠻⣕⡦⢌⣑⠲⠤⣄⣈⠙⠢⢤⣀⡀⠀⢸⠏⣠⠈⠸⣟⣎⣄⡴⠚⠁⠀⠀⢀⣿⠟⣧⡿⠛⣶⣦⠤⠤⠶⣾⠛⠉⠀⠀⠀⠻⣟⠳⢤
⠀⡜⣾⣿⣿⣿⣿⣿⣿⣿⢹⢻⣇⢠⡏⠀⢳⡐⠃⣰⠏⣷⠀⠀⠀⠙⠳⠮⣏⡲⠒⠭⠭⠍⣒⡒⡻⠿⣟⣋⣁⣀⠤⠀⠋⠁⠀⠀⠀⠀⠤⠞⢁⣴⠟⣁⡼⠋⠈⣇⠀⠀⠙⣆⠀⠀⠀⠀⠀⠙⢧⡀
⠀⠻⣛⠿⣿⣿⣿⣿⣿⣿⡏⡎⢿⣾⣱⡔⣌⢷⣜⠁⠀⡇⡂⠀⠀⣠⣴⣒⣒⣉⣙⠓⠒⠒⠤⠴⠔⠛⠛⠋⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⠴⣿⣡⣾⣿⡀⠀⠀⢹⠀⠀⠀⢹⣆⠀⠀⠀⠀⠀⠈⢳
⠀⠀⠈⠉⠐⠚⠫⠽⠿⠿⢧⠸⡜⣾⣇⠹⡜⢮⢳⡵⠄⣇⠡⢤⣾⣿⣿⣿⡿⣿⣯⡉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠤⠴⠟⠋⠁⡿⣿⣧⠀⠀⠸⣿⠐⣰⢻⣿⡆⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡀⣷⡈⢿⡆⠙⢦⡳⣿⣦⣻⡤⣿⡿⠻⣗⣤⣯⡀⠻⢷⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⠃⢿⣹⡄⠀⠀⣿⠀⣿⠀⡟⣿⡀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⣇⢷⢳⡀⠙⢤⠀⠉⠚⠻⣿⣾⢿⡧⣀⠈⠺⣷⣇⣠⠞⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⠏⠀⢸⡏⢷⠀⠀⣿⡄⢹⠀⢧⠸⣇⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⡼⡆⠳⡄⠀⢕⢤⡀⠀⠈⠻⢷⣍⡑⠋⠶⠲⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⠔⡿⠀⠀⠀⠀⠀⣠⡏⠀⠀⠀⣿⣜⣧⢠⡏⠁⠸⠀⠈⠇⣻⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻⣿⡀⠙⢦⡀⠙⢿⣶⣤⠀⠀⠉⠻⣦⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⡤⠒⠉⢀⡼⠁⠀⠀⠀⠀⣴⠋⠀⣀⡴⡞⠛⣿⡼⣾⡇⠀⠀⠀⠀⢸⠘⡧⡀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠻⣿⣰⡷⠛⢦⡈⠓⢍⠻⣶⢦⣤⣌⣻⣷⣦⣤⡤⠀⠀⠀⠀⠀⠀⠀⠉⠒⠒⠋⠁⠀⠀⠀⠀⢀⣴⠃⣠⡾⠋⠁⠀⠀⢻⣿⣾⠃⠀⡅⠀⠀⣼⡀⣷⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣼⡿⢁⠀⠈⣿⢲⣤⢕⡺⣷⣇⢸⠀⠀⢸⡲⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡞⣡⡾⠉⠀⠀⠀⠀⠀⠈⢻⣿⡆⢰⠁⠀⡴⡏⡇⢹⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣖⡿⠃⠀⠀⠀⢀⣾⠀⠈⡟⠺⣭⣟⣻⢤⠀⠀⡇⣿⠋⠙⢶⣶⡤⠤⠤⠤⣤⣤⣀⣀⣀⣀⡴⠋⣼⠟⠀⠀⠀⠀⠀⠀⠀⣠⣾⡇⡸⡟⠀⣘⡰⡇⠇⢸⡇⠀⠀⠀
⠠⠤⣚⣛⠷⣤⠀⠀⣀⢤⣺⣿⡏⠀⠀⠀⢀⡴⠋⢸⠀⠀⠳⢸⠃⢸⢹⡉⠉⠉⠀⢸⠀⠀⠈⣿⡀⠀⠀⠀⠀⠀⠈⣿⣧⡀⢀⡾⠃⠀⠀⠀⠀⠀⠀⢀⣾⣿⡿⣄⡿⡴⠉⠀⠀⡇⠀⢸⣷⣶⣶⣤
⠄⠀⠉⠉⢉⣉⣩⣭⣷⣿⣿⣿⣅⣀⡤⠚⠁⠀⠀⣼⠀⠀⠀⣼⠀⣼⠐⡇⠀⢀⠀⣾⠀⠀⠀⡇⡇⠀⠀⠀⠀⣠⣿⣿⣿⣷⡞⠁⠀⠀⠀⠀⠀⠀⣰⣿⣿⣿⢃⡹⡼⠀⠀⠀⠀⣿⠀⣼⣿⣿⡿⠛
⠃⠀⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠀⠀⠀⠀⠀⠀⠀⡏⠀⠀⢀⡏⠀⡟⡀⢷⠀⡾⢀⡿⠀⠀⠀⢡⢸⠀⣀⣴⣾⣿⣿⣿⣿⣿⣗⣪⡖⢶⡆⠀⢀⣾⣿⣿⣿⠏⢡⠳⠁⠀⠀⠀⢰⢻⣠⠇⣿⠟⠁⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⠃⠀⢀⡾⠀⢠⣷⡇⠸⡶⡇⣸⠃⠀⠀⠀⣘⣼⣾⣿⣿⣿⣿⡿⣻⣛⣿⣭⡿⢷⠾⠃⣠⣾⣿⠿⢫⡏⢀⡏⠀⠀⠀⠀⢀⡪⣿⠸⢸⡇⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⠏⠀⣠⠟⠀⠀⣼⣾⢱⡀⢻⣡⡏⠀⠀⣰⣿⣿⣿⣿⣿⣿⣿⠃⠻⠟⣹⣿⣷⠀⢀⣴⡿⣿⠟⠁⣤⠟⢀⡞⠀⠀⠀⠀⠔⠋⢠⡇⠁⣾⠇⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠟⣠⠾⠃⠀⠀⢠⡇⣿⣈⢧⠈⢿⣇⣴⣿⣿⣿⣿⣿⣿⢟⣿⠇⠀⢀⣼⣿⣿⣿⣷⠿⣫⠞⠁⣠⣾⠋⢠⡞⠀⠀⠀⠀⠀⠀⠀⣼⡅⢠⣿⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⣾⠵⠊⠁⠀⠀⠀⠀⣼⢱⣿⡈⢪⢧⡈⢿⣱⣿⣿⣿⣿⣿⢫⢸⣿⠀⣤⠟⠋⢡⣴⠟⡡⠊⢡⡠⣾⠟⢁⡴⠛⠀⠀⠀⠀⠀⢀⠀⣰⡟⢀⣼⡏⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡟⢸⡿⡇⠀⠈⢳⡈⢿⣿⣿⣿⠋⠀⢸⠈⣿⡟⠂⣠⡾⢋⡴⠊⠀⠚⣻⠞⢁⣴⠟⢀⠔⠁⠀⠀⠀⢀⠌⣴⣿⡡⠾⠋⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⡇⠀⡇⢳⠀⠀⠀⠹⢄⢻⡟⠁⠀⠀⢸⠀⢻⣧⡾⢋⣐⣋⣀⣬⣔⣯⠴⢺⡟⠁⠀⠀⠀⠀⠀⢀⡠⠮⢞⡟⠉⠘⠁⠀⠀⠀⠀⠀⠀
*/
