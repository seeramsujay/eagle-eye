// =============================================================================
// Project Eagle-Eye: 25° Side-Angled Aerodynamic Nose Camera Mount
// 25° Downward Tilt + 15° Lateral Left Orbit Offset with Protective Crash Hood
// Standard 19x19mm Micro FPV Camera (RunCam / Foxeer / Caddx / Analog / Digital)
// =============================================================================

$fn = 36;

cam_width       = 19.4;  // Micro camera 19mm width + tolerance
cam_depth       = 19.4;  // Micro camera height
tilt_down_deg   = 25.0;  // 25° downward pitch for 40-60m altitude tracking
pan_left_deg    = 15.0;  // 15° left yaw to center moving car during left-hand orbit
wall_thick      = 2.2;
hood_overhang   = 14.0;  // Extends ahead of lens to absorb ground impact

module camera_mount() {
    difference() {
        // Outer Aerodynamic Nose Block
        hull() {
            // Rear mating flange (fits snugly into fuselage pod nose)
            translate([-15, 0, 0])
                scale([1, 1, 0.85])
                    cylinder(r = 25, h = 32, center = true);

            // Forward protective lens hood
            translate([18, -4, -6])
                rotate([0, tilt_down_deg, pan_left_deg])
                    cube([cam_depth + wall_thick * 2, cam_width + wall_thick * 2 + 6, 26], center = true);
        }

        // Camera Pocket with Compound 25° Down + 15° Left Angle
        translate([12, -4, -4]) {
            rotate([0, tilt_down_deg, pan_left_deg]) {
                // Internal camera body cutout
                cube([cam_depth + 4, cam_width, 24], center = true);

                // Front lens aperture
                translate([10, 0, 0])
                    cylinder(r = 8.5, h = 25, center = true); // 17mm lens aperture

                // Side M2 mounting screw slots (for fine tilt adjustment)
                for (side = [-1, 1]) {
                    translate([0, side * (cam_width / 2), 0])
                        rotate([90, 0, 0])
                            hull() {
                                translate([0, 3, 0]) cylinder(r = 1.25, h = 10, center = true);
                                translate([0, -3, 0]) cylinder(r = 1.25, h = 10, center = true);
                            }
                }

                // Rear wire routing channel
                translate([-15, 0, 0])
                    cube([18, 12, 12], center = true);
            }
        }

        // Weight Reduction & Air Cooling Core
        translate([-10, 0, -5])
            scale([1, 0.75, 0.75])
                sphere(r = 16);

        // Mating screw holes to secure nose onto fuselage pod
        for (side = [-1, 1]) {
            translate([-8, side * 18, 0])
                rotate([90, 0, 0])
                    cylinder(r = 1.5, h = 15, center = true); // 3mm mounting screw
        }
    }

    // Impact skid bumper ridge on underside
    translate([5, -2, -18])
        hull() {
            cube([22, 12, 3], center = true);
            translate([8, 0, 2])
                cube([10, 8, 2], center = true);
        }
}

camera_mount();
