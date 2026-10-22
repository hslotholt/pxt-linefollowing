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
    
    // Initialize sensor and select line colour before calling this block.
    // Stops immediately if no line is detected. Forward only.
    //% blockId="robot_follow_line"
    //% block="Start linjefølgning med $speed mm/s"
    //% speed.defl=120 speed.min=1
    export function start(speed: number): void {
        if (speed <= 0) { nezhaV2_odometry.stop(); return; }
        nezhaV2_odometry.startExternalController(function () {
            LineSensor8Pixel.sample();
            if (!LineSensor8Pixel.hasLine()) return false;
            let position = Math.max(-100, Math.min(100, LineSensor8Pixel.getLinePos()));
            let offset = position * halfWidthMm / 100;
            let curvature = -steeringGain * 2 * offset /
                (forwardMm * forwardMm + offset * offset);
            curvature = Math.max(-1 / 150, Math.min(1 / 150, curvature));
            nezhaV2_odometry.setExternalVelocity(speed, speed * curvature);
            return true;
        });
    }
}
