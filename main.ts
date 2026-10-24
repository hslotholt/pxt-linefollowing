//% color=#2876A8 icon="\uf018" block="Linjefølgning"
namespace robotLineFollowing {
    let forwardMm = 150;
    let halfWidthMm = 66.5;
    let steeringGain = 1;

    //% advanced=true blockId="robot_line_setup"
    //% block="Indstil sensorafstand $distance mm bredde $width mm styrefaktor $gain"
    //% distance.defl=150 width.defl=133 gain.defl=1
    export function configure(distance: number, width: number, gain: number): void {
        if (distance > 0 && width > 0 && gain >= 0) {
            forwardMm = distance;
            halfWidthMm = width / 2;
            steeringGain = gain;
        }
    }

    // mode: 0 continuous, 1 until junction, 2 until line lost, 3 distance.
    function beginFollowing(speed: number, mode: number, distance: number): void {
        if (speed <= 0 || (mode == 3 && distance <= 0)) {
            nezhaV2_odometry.stop();
            return;
        }
        // Idempotent: initialize only if needed. Acquisition then continues
        // independently of this adapter and every robot movement.
        LineSensor8Pixel.initLinesensor8Pixel();
        let started = input.runningTime();
        let startTrip = nezhaV2_odometry.getTrip();
        let startCount = LineSensor8Pixel.getJunctionCount();
        let seenLine = false;
        let success = false;
        let pendingSince = -1;
        nezhaV2_odometry.startExternalController(function () {
            let now = input.runningTime();
            if (LineSensor8Pixel.getSampleAgeMs() > 100) {
                if (!seenLine && now - started < 250) {
                    nezhaV2_odometry.setExternalVelocity(0, 0);
                    return true;
                }
                return false;
            }
            // Retained count captures an event even if it passed between callbacks.
            if (mode == 1 && (LineSensor8Pixel.isAtJunction() ||
                LineSensor8Pixel.getJunctionCount() != startCount)) {
                success = true;
                return false;
            }
            let remaining = distance - (nezhaV2_odometry.getTrip() - startTrip);
            if (mode == 3 && remaining <= 2) {
                success = true;
                return false;
            }
            if (!LineSensor8Pixel.hasLine()) {
                // Give a broad-colour transition time to confirm as a possible
                // crossing before classifying it as a failure in junction mode.
                if (mode == 1 && LineSensor8Pixel.isJunctionCandidate()) {
                    if (pendingSince < 0) pendingSince = now;
                    if (now - pendingSince < 100) {
                        nezhaV2_odometry.setExternalVelocity(0, 0);
                        return true;
                    }
                }
                success = mode == 2 && seenLine;
                return false;
            }
            pendingSince = -1;
            seenLine = true;
            if (mode == 3 && now - started > 5000 + 4000 * distance / speed) return false;
            let position = Math.max(-100, Math.min(100, LineSensor8Pixel.getLinePos()));
            let offset = position * halfWidthMm / 100;
            let curvature = steeringGain * 2 * offset /
                (forwardMm * forwardMm + offset * offset);
            curvature = Math.max(-1 / 100, Math.min(1 / 100, curvature));
            let requestedSpeed = mode == 3 ? Math.min(speed, Math.max(0, 3 * remaining)) : speed;
            nezhaV2_odometry.setExternalVelocity(requestedSpeed, requestedSpeed * curvature);
            return true;
        }, function () { return success; });
    }

    // Sensor initialization and colour/branch selection belong in startup.
    //% blockId="robot_follow_line"
    //% block="Start linjefølgning med $speed mm/s"
    //% speed.defl=120 speed.min=1
    export function start(speed: number): void { beginFollowing(speed, 0, 0); }

    //% blockId="robot_follow_until_junction"
    //% block="Følg linje indtil udgrening eller kryds med $speed mm/s"
    //% speed.defl=120 speed.min=1
    export function followUntilJunction(speed: number): void {
        beginFollowing(speed, 1, 0);
        nezhaV2_odometry.waitUntilMotionFinished();
    }

    // A line must have been seen first. Starting with no line is a failure.
    //% blockId="robot_follow_until_lost"
    //% block="Følg linje indtil ingen linje med $speed mm/s"
    //% speed.defl=120 speed.min=1
    export function followUntilLost(speed: number): void {
        beginFollowing(speed, 2, 0);
        nezhaV2_odometry.waitUntilMotionFinished();
    }

    // Distance refers to axle centre travel, not sensor position.
    //% blockId="robot_follow_distance"
    //% block="Følg linje $distance mm med $speed mm/s"
    //% distance.defl=500 distance.min=1 speed.defl=120 speed.min=1
    export function followDistance(distance: number, speed: number): void {
        beginFollowing(speed, 3, distance);
        nezhaV2_odometry.waitUntilMotionFinished();
    }
}
