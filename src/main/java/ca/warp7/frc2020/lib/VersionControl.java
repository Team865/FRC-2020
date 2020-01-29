package ca.warp7.frc2020.lib;

public class VersionControl {
    public static void printVersionInfo() {
        System.out.println("Build Date: " + BuildConfig.kDeployTime);
        System.out.println("Deploy User: " + BuildConfig.kDeployUser);
        System.out.println("Latest Commit: " + BuildConfig.kGitRevision);
    }
}
