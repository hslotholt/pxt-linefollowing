//% color=#2876A8 icon="\uf018" block="Linjefølgning"
namespace robotLineFollowing {
    export enum LineStopCondition {
        //% block="aldrig/distance"
        Never = 0,
        //% block="udgrening"
        Junction = 1,
        //% block="kryds"
        Crossing = 2,
        //% block="udgrening eller kryds"
        JunctionOrCrossing = 3,
        //% block="ingen linje"
        LineLost = 4,
        //% block="ikke længere ren linje"
        NotCleanLine = 5
    }

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

    function beginFollowing(speed: number, stopCondition: LineStopCondition,
        distance: number, distanceLimited: boolean): void {
        if (speed <= 0 || (distanceLimited && distance <= 0)) {
            nezhaV2_odometry.stop();
            return;
        }
        LineSensor8Pixel.initLinesensor8Pixel();
        let started = input.runningTime();
        let startTrip = 0;
        let startJunctionCount = LineSensor8Pixel.getJunctionCount();
        let startBranchCount = LineSensor8Pixel.getBranchCount();
        let startCrossingCount = LineSensor8Pixel.getCrossingCount();
        let startLossCount = LineSensor8Pixel.getLineLossCount();
        let startUncleanCount = LineSensor8Pixel.getNotCleanLineCount();
        let success = false;
        let receivedSample = false;
        let firstRun = true;
        let lastCurvature = 0;
        nezhaV2_odometry.startExternalController(function () {
            if (firstRun) {
                startTrip = nezhaV2_odometry.getTrip();
                firstRun = false;
            }
            let now = input.runningTime();
            if (LineSensor8Pixel.getSampleAgeMs() > 100) {
                // Missing samples are faults, never normal "line lost" events.
                if (!receivedSample && now - started < 250) {
                    nezhaV2_odometry.setExternalVelocity(0, 0);
                    return true;
                }
                return false;
            }
            receivedSample = true;
            let reached = false;
            if (stopCondition == LineStopCondition.Junction) {
                reached = LineSensor8Pixel.isAtBranch() ||
                    LineSensor8Pixel.getBranchCount() != startBranchCount;
            } else if (stopCondition == LineStopCondition.Crossing) {
                reached = LineSensor8Pixel.isOnCrossing() ||
                    LineSensor8Pixel.getCrossingCount() != startCrossingCount;
            } else if (stopCondition == LineStopCondition.JunctionOrCrossing) {
                reached = LineSensor8Pixel.isAtJunction() ||
                    LineSensor8Pixel.getJunctionCount() != startJunctionCount;
            } else if (stopCondition == LineStopCondition.LineLost) {
                reached = LineSensor8Pixel.isLineLostConfirmed() ||
                    LineSensor8Pixel.getLineLossCount() != startLossCount;
            } else if (stopCondition == LineStopCondition.NotCleanLine) {
                reached = LineSensor8Pixel.isNotCleanLineConfirmed() ||
                    LineSensor8Pixel.getNotCleanLineCount() != startUncleanCount;
            }
            if (reached) {
                // If stopped before distance reached, success = false
                success = !distanceLimited;
                return false;
            }
            let remaining = distance - (nezhaV2_odometry.getTrip() - startTrip);
            if (distanceLimited && remaining <= 2) {
                success = true;
                return false;
            }
            if (distanceLimited && now - started > 5000 + 4000 * distance / speed) return false;
            if (LineSensor8Pixel.hasLine()) {
                let position = Math.max(-100, Math.min(100, LineSensor8Pixel.getLinePos()));
                let offset = position * halfWidthMm / 100;
                lastCurvature = steeringGain * 2 * offset /
                    (forwardMm * forwardMm + offset * offset);
                lastCurvature = Math.max(-1 / 100, Math.min(1 / 100, lastCurvature));
            }
            // If the line is absent, keep the last valid steering until the
            // selected confirmed stop condition occurs. Never means continue;
            // before any valid line has been seen, the default is straight.
            let requestedSpeed = distanceLimited ? Math.min(speed, Math.max(0, 3 * remaining)) : speed;
            nezhaV2_odometry.setExternalVelocity(requestedSpeed, requestedSpeed * lastCurvature);
            return true;
        }, function () { return success; });
    }

    //% blockId="robot_follow_line"
    //% block="Start linjefølgning med hastighed $speed mm/s stop ved $stopCondition"
    //% speed.defl=120 speed.min=1 stopCondition.defl=robotLineFollowing.LineStopCondition.Never
    export function start(speed: number, stopCondition: LineStopCondition = LineStopCondition.Never): void {
        beginFollowing(speed, stopCondition, 0, false);
        if (stopCondition == LineStopCondition.Never) return;
        waitUntilFinished();
    }

    //% blockId="robot_line_wait" block="Vent til linje følgning er afsluttet"
    export function waitUntilFinished(): void {
        nezhaV2_odometry.waitUntilMotionFinished();
    }

    //% blockId="robot_follow_distance"
    //% block="Følg linje i $distance mm med hastighed $speed mm/s | stop hvis $stopCondition"
    //% distance.defl=500 distance.min=1 speed.defl=120 speed.min=1
    export function followDistance(distance: number, speed: number, stopCondition: LineStopCondition = LineStopCondition.Never): void {
        beginFollowing(speed, stopCondition, distance, true);
        waitUntilFinished();
    }
}
