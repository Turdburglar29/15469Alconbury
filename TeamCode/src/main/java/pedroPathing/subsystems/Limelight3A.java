package pedroPathing.subsystems;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

    /**
     * Limelight 3A helper: reads AprilTag-based global robot pose from http://<ip>/json
     * Returns pose in inches and radians for FTC-friendly use.
     * Designed for short timeouts so TeleOp loop stays responsive.
     */
    public class Limelight3A{

        private final String limelightIP;

        private double lastX_m = 0, lastY_m = 0; // meters
        private double lastHeadingRad = 0;
        private boolean hasValidTarget = false;

        public Limelight3A(String ipAddress) {
            this.limelightIP = ipAddress;
        }

        /** Call per loop (or per sample in a burst). Non-fatal if connection fails. */
        public void update() {
            try {
                URL url = new URL("http://" + limelightIP + "/json");
                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                conn.setConnectTimeout(25);
                conn.setReadTimeout(25);

                BufferedReader reader = new BufferedReader(new InputStreamReader(conn.getInputStream()));
                StringBuilder data = new StringBuilder();
                String line;
                while ((line = reader.readLine()) != null) data.append(line);
                reader.close();

                JSONObject json = new JSONObject(data.toString());
                JSONObject results = json.getJSONObject("Results");

                if (!results.has("FiducialID")) { hasValidTarget = false; return; }
                JSONArray fids = results.getJSONArray("FiducialID");
                hasValidTarget = (fids.length() > 0);
                if (!hasValidTarget) return;

                // BotPose format: [X, Y, Z, roll, pitch, yaw(deg)] in meters/degrees
                JSONArray botpose = results.getJSONArray("BotPose");
                lastX_m = botpose.getDouble(0);
                lastY_m = botpose.getDouble(1);
                double yawDeg = botpose.getDouble(5);
                lastHeadingRad = Math.toRadians(yawDeg);

            } catch (Exception e) {
                hasValidTarget = false; // connection or parse error
            }
        }

        public boolean hasValidTarget() { return hasValidTarget; }

        public double getX_in() { return lastX_m * 39.3701; }
        public double getY_in() { return lastY_m * 39.3701; }
        public double getHeadingRad() { return lastHeadingRad; }
    }


