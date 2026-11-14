// =============================================================================
// Project Eagle-Eye: Parametric Streamlined Fuselage Pod
// Avionics Pod with Vibration-Isolated IMU Tray, Top GPS Pedestal,
// Vertical LoRa SMA Boss, Spar Pass-Throughs, and 2° Down/Right Motor Firewall
// =============================================================================

$fn = 40;

// Key Dimensions (mm)
pod_length     = 180;
pod_width      = 54;
pod_height     = 42;
wall_thick     = 2.2;
floor_thick    = 2.4;

// Motor Mount Specifications (A2212 Outrunner)
motor_bolt_dia = 3.2;   // M3 clearance
motor_pitch_a  = 16.0;  // 16mm hole pattern
motor_pitch_b  = 19.0;  // 19mm hole pattern
motor_shaft_dia= 11.0;  // Center shaft clearance
thrust_down_deg= 2.0;   // 2° down thrust offset
thrust_right_deg=2.0;   // 2° right thrust offset

// GPS Pedestal (Elevated to isolate ceramic patch from ESC RF EMI)
gps_ped_height = 28.0;
gps_tray_w     = 28.0;
gps_tray_l     = 28.0;

// Spar Pass-Through
spar_dia       = 4.4;   // 4mm bamboo/carbon spar clearance

module fuselage_shell() {
    difference() {
        // Outer Aerodynamic Pod Body
        hull() {
            // Nose section
            translate([15, 0, pod_height/2])
                sphere(r = pod_width/2 - 2);
            // Mid fuselage body
            translate([pod_length * 0.45, 0, pod_height/2])
                scale([1.8, 1, 0.85])
                    cylinder(r = pod_width/2, h = 2, center = true);
            // Rear motor firewall transition
            translate([pod_length - 8, 0, pod_height/2])
                scale([0.8, 0.75, 0.75])
                    cube([12, pod_width, pod_height], center = true);
        }

        // Inner Electronics & Battery Cavity
        hull() {
            translate([22, 0, pod_height/2 + floor_thick])
                scale([1, 0.88, 0.85])
                    sphere(r = pod_width/2 - wall_thick - 2);
            translate([pod_length * 0.45, 0, pod_height/2 + floor_thick])
                scale([1.7, 0.88, 0.75])
                    cylinder(r = pod_width/2 - wall_thick, h = 2, center = true);
            translate([pod_length - 16, 0, pod_height/2 + floor_thick])
                cube([10, pod_width - (wall_thick * 2 + 6), pod_height - (wall_thick * 2 + 4)], center = true);
        }

        // Top Hatch Opening for Quick Battery / Avionics Access
        translate([pod_length * 0.45, 0, pod_height - 2])
            cube([pod_length * 0.65, pod_width - (wall_thick * 2 + 4), 15], center = true);

        // Wing Spar Pass-Through Holes (Left & Right)
        translate([pod_length * 0.52, 0, pod_height * 0.38])
            rotate([90, 0, 0])
                cylinder(r = spar_dia / 2, h = pod_width + 20, center = true);

        translate([pod_length * 0.70, 0, pod_height * 0.45])
            rotate([90, 0, 0])
                cylinder(r = spar_dia / 2, h = pod_width + 20, center = true);

        // NACA Air Cooling Inlets (Front Sides)
        for (side = [-1, 1]) {
            translate([35, side * (pod_width/2 - 2), pod_height * 0.4])
                rotate([0, 15, side * 20])
                    cube([18, 5, 8], center = true);
        }
    }
}

// Elevated GPS Pedestal with Hollow Wire Duct
module gps_pedestal() {
    translate([pod_length * 0.28, 0, pod_height - 4]) {
        difference() {
            union() {
                // Tapered aerodynamic mast
                cylinder(r1 = 9, r2 = 6, h = gps_ped_height);
                // Top receiver tray for NEO-6M 25x25 ceramic patch
                translate([0, 0, gps_ped_height])
                    hull() {
                        cube([gps_tray_l + 3, gps_tray_w + 3, 3], center = true);
                        translate([0, 0, 2])
                            cube([gps_tray_l + 1, gps_tray_w + 1, 2], center = true);
                    }
            }
            // Central wire pass-through hole (5.5mm)
            translate([0, 0, -2])
                cylinder(r = 2.8, h = gps_ped_height + 10);
            // Patch antenna recess (26 x 26 x 3mm)
            translate([0, 0, gps_ped_height + 1.5])
                cube([26.0, 26.0, 3.5], center = true);
        }
    }
}

// Vertical LoRa SMA Antenna Boss with Hex Nut Recess
module lora_sma_boss() {
    translate([pod_length * 0.75, 12, pod_height - 3]) {
        difference() {
            cylinder(r = 6.0, h = 10.0);
            // 6.5mm hole for standard SMA bulkhead connector
            translate([0, 0, -2])
                cylinder(r = 3.25, h = 15.0);
            // 8mm hex anti-rotation seat
            translate([0, 0, -1])
                cylinder(r = 4.6, h = 3.5, $fn = 6);
        }
    }
}

// Vibration-Isolated IMU Tray Stanchions (4x M2.5 / Silicone Grommets)
module imu_tray_stanchions() {
    translate([pod_length * 0.42, 0, floor_thick]) {
        for (dx = [-11, 11]) {
            for (dy = [-9, 9]) {
                translate([dx, dy, 0]) {
                    difference() {
                        cylinder(r = 3.2, h = 6.0);
                        translate([0, 0, 1.5])
                            cylinder(r = 1.3, h = 6.0); // 2.6mm hole for self-tapping / grommet
                    }
                }
            }
        }
    }
}

// Rear Motor Firewall with 2° Down / 2° Right Thrust Offset
module motor_firewall() {
    translate([pod_length - 4, 0, pod_height/2 - 2]) {
        // Compound angle offset: rotate for down and right thrust
        rotate([0, thrust_down_deg, thrust_right_deg]) {
            difference() {
                // Solid bulkhead plate (4.5mm thick)
                cube([5.0, pod_width * 0.68, pod_height * 0.75], center = true);

                // Center motor shaft & bearing recess
                rotate([0, 90, 0])
                    cylinder(r = motor_shaft_dia / 2, h = 12, center = true);

                // 4x Motor Mounting Bolt Holes (Cross Pattern: 16mm & 19mm)
                for (angle = [0, 90, 180, 270]) {
                    rotate([angle, 0, 0]) {
                        // 16mm pattern on horizontal, 19mm on vertical
                        pitch = (angle == 0 || angle == 180) ? motor_pitch_a / 2 : motor_pitch_b / 2;
                        translate([0, pitch, 0])
                            rotate([0, 90, 0])
                                cylinder(r = motor_bolt_dia / 2, h = 12, center = true);
                    }
                }

                // Motor cooling exhaust vents
                for (a = [45, 135, 225, 315]) {
                    rotate([a, 0, 0])
                        translate([0, 13.5, 0])
                            rotate([0, 90, 0])
                                cylinder(r = 2.2, h = 12, center = true);
                }
            }
        }
    }
}

// Assembled Printable Avionics Fuselage Pod
module complete_fuselage_pod() {
    union() {
        fuselage_shell();
        gps_pedestal();
        lora_sma_boss();
        imu_tray_stanchions();
        motor_firewall();
    }
}

complete_fuselage_pod();
