// =============================================================================
// Project Eagle-Eye v0 Micro: 18650 Twin-Coreless FPV Flying Wing (300mm Span)
// Designed for:
// - 1S/2S 18650 Li-ion Cell (18.5mm x 65.5mm)
// - FPV Camera (14mm Nano / 19mm Micro) with 20° downward tilt in nose
// - ESP32-WROOM-32 DevKit + L293D Motor Driver
// - Twin 8520 / 716 Coreless Motors in wings with 55-65mm props
// - Differential Thrust Steering (Zero servos required)
// Fits diagonally on standard 220x220mm print bed
// =============================================================================

$fn = 32;

// Wingspan and Planform (mm)
span            = 300.0;
half_span       = span / 2.0;
root_chord      = 115.0;
tip_chord       = 55.0;
sweep_angle_deg = 24.0;
dihedral_deg    = 3.0;

// Motor Nacelles (8520 / 716 Coreless Motors)
motor_dia       = 8.8;   // 8.5mm motor + tolerance (accommodates 8520 or 716 with spacer)
motor_len       = 21.0;  // 20.0mm length
nacelle_y_pos   = 68.0;  // Distance from centerline to motor axis on each wing
prop_dia        = 65.0;  // Up to 65mm propeller clearance

// Center Fuselage Pod Dimensions
pod_len         = 135.0;
pod_width       = 38.0;
pod_height      = 28.0;

// 18650 Battery Bay Dimensions
cell_dia        = 18.8;
cell_len        = 66.0;

// Camera Pocket (Micro/Nano FPV Camera)
cam_width       = 19.4;
cam_height      = 19.4;
cam_tilt_deg    = 20.0;

// 2D Reflexed Airfoil Profile (S-Camber for Pitch Stability)
module reflex_airfoil_2d(chord, thickness_ratio = 0.085) {
    t = chord * thickness_ratio;
    hull() {
        // Leading edge
        translate([chord * 0.12, 0])
            circle(r = t * 0.45);
        // Thickness crest (28% chord)
        translate([chord * 0.28, t * 0.12])
            circle(r = t * 0.50);
        // Reflexed camber lift (85% chord)
        translate([chord * 0.85, t * 0.20])
            circle(r = t * 0.16);
        // Trailing edge
        translate([chord - 1.0, t * 0.25])
            square([1.5, 0.8], center = true);
    }
}

// Single Wing Half with Integrated Motor Nacelle and Tip Fin
module wing_half(is_right = true) {
    dir = is_right ? 1 : -1;
    dx_tip = half_span * tan(sweep_angle_deg);
    dz_tip = half_span * sin(dihedral_deg);

    difference() {
        union() {
            // Wing Solid Loft
            hull() {
                // Root
                translate([0, dir * (pod_width / 2 - 2), 0])
                    rotate([90, 0, 0])
                        linear_extrude(height = 3.0, center = true)
                            reflex_airfoil_2d(root_chord);

                // Mid
                translate([dx_tip * 0.5, dir * (half_span * 0.5), dz_tip * 0.5])
                    rotate([90, 0, 0])
                        linear_extrude(height = 2.0, center = true)
                            reflex_airfoil_2d(root_chord * 0.72);

                // Tip
                translate([dx_tip, dir * (half_span - 2.0), dz_tip])
                    rotate([90, 0, 0])
                        linear_extrude(height = 2.0, center = true)
                            reflex_airfoil_2d(tip_chord);
            }

            // Wingtip Vertical Stabilizer (Winglet)
            translate([dx_tip + tip_chord * 0.38, dir * (half_span - 1.0), dz_tip + 16.0]) {
                rotate([0, -8, dir * 1.5])
                    hull() {
                        cube([tip_chord * 0.85, 1.6, 2], center = true);
                        translate([tip_chord * 0.18, 0, 28.0])
                            cube([tip_chord * 0.35, 1.2, 2], center = true);
                    }
            }

            // Integrated Coreless Motor Nacelle Pod
            nacelle_x = (nacelle_y_pos / half_span) * dx_tip + 26.0;
            nacelle_z = (nacelle_y_pos / half_span) * dz_tip;

            translate([nacelle_x, dir * nacelle_y_pos, nacelle_z]) {
                hull() {
                    translate([0, 0, 0])
                        sphere(r = (motor_dia / 2) + 2.0);
                    translate([motor_len + 4, 0, 0])
                        cylinder(r = (motor_dia / 2) + 1.6, h = 2, center = true);
                }
            }
        }

        // Motor Nacelle Internal Cavity
        nacelle_x = (nacelle_y_pos / half_span) * dx_tip + 26.0;
        nacelle_z = (nacelle_y_pos / half_span) * dz_tip;

        translate([nacelle_x, dir * nacelle_y_pos, nacelle_z]) {
            // Motor bore
            translate([4, 0, 0])
                rotate([0, 90, 0])
                    cylinder(r = motor_dia / 2, h = motor_len + 12, center = true);

            // Rear prop shaft clearance
            translate([motor_len + 5, 0, 0])
                rotate([0, 90, 0])
                    cylinder(r = 3.6, h = 10, center = true);

            // Wire routing tunnel leading into center fuselage
            translate([-10, 0, 0])
                rotate([0, 90, dir * 48])
                    cylinder(r = 1.6, h = 30, center = true);

            // Motor cooling slot
            translate([motor_len * 0.45, 0, (motor_dia / 2) + 1.0])
                cube([10.0, 2.8, 4.0], center = true);
        }

        // Weight Reduction Cavities
        translate([dx_tip * 0.42 + 22, dir * (half_span * 0.42), dz_tip * 0.42])
            scale([1, 1, 0.65])
                sphere(r = 18);

        translate([dx_tip * 0.72 + 18, dir * (half_span * 0.72), dz_tip * 0.72])
            scale([1, 1, 0.60])
                sphere(r = 14);
    }
}

// Center Fuselage Pod with Camera, 18650 Battery, ESP32, and L293D Bays
module center_fuselage_18650() {
    difference() {
        // Outer Aerodynamic Shell
        hull() {
            // Nose cone
            translate([12, 0, 0])
                sphere(r = pod_width / 2);
            // Mid pod body
            translate([pod_len * 0.48, 0, 0])
                scale([1.3, 1.0, 0.88])
                    cylinder(r = pod_width / 2, h = pod_height, center = true);
            // Rear tail transition
            translate([pod_len - 6, 0, 2])
                scale([0.8, 0.65, 0.65])
                    cube([10, pod_width, pod_height], center = true);
        }

        // 1. Nose FPV Camera Pocket (19mm Micro / 14mm Nano with 20° tilt)
        translate([16, 0, 2]) {
            rotate([0, cam_tilt_deg, 0]) {
                // Camera body cutout
                cube([22, cam_width, cam_height], center = true);
                // Lens aperture
                translate([12, 0, 0])
                    cylinder(r = 8.5, h = 20, center = true);
                // Side M2 mounting slots
                for (side = [-1, 1]) {
                    translate([0, side * (cam_width / 2), 0])
                        rotate([90, 0, 0])
                            cylinder(r = 1.25, h = 10, center = true);
                }
            }
        }

        // 2. 18650 Battery Cradle (Positioned precisely on CG at 35-50% chord)
        translate([pod_len * 0.46, 0, -pod_height * 0.22]) {
            rotate([0, 90, 0])
                cylinder(r = cell_dia / 2, h = cell_len, center = true);
            // Top insertion & finger ejection slot
            translate([0, 0, cell_dia * 0.35])
                cube([cell_len - 12, cell_dia * 0.85, cell_dia], center = true);
            // Underbelly finger push hole
            translate([0, 0, -cell_dia * 0.45])
                cube([cell_len - 24, 8.0, 8.0], center = true);
        }

        // 3. Upper Avionics Bay (ESP32-WROOM DevKit + L293D DIP-16 + MPU6050)
        translate([pod_len * 0.52, 0, pod_height * 0.25])
            cube([pod_len * 0.62, pod_width - 4.0, pod_height * 0.45], center = true);

        // 4. L293D DIP-16 Chip Recess on Mid Deck
        translate([pod_len * 0.72, 0, pod_height * 0.08])
            cube([22.0, 10.5, 6.0], center = true);

        // 5. MPU6050 Sensor Tray directly at CG
        translate([pod_len * 0.46, 0, pod_height * 0.05])
            cube([22.0, 16.0, 4.0], center = true);

        // 6. Left & Right Wing Motor Wire Ports
        for (side = [-1, 1]) {
            translate([pod_len * 0.60, side * (pod_width / 2), 2])
                rotate([90, 0, 0])
                    cylinder(r = 2.5, h = 10, center = true);
        }

        // 7. Top Cooling Air Scoop & Hatch Access
        translate([32, 0, pod_height * 0.42])
            cube([16, 8, 8], center = true);
    }

    // Underside Landing Skid Bumpers
    for (dx = [25, pod_len - 25]) {
        translate([dx, 0, -pod_height / 2 - 1.0])
            hull() {
                cube([14, 6, 2], center = true);
                translate([0, 0, 1]) cube([8, 4, 1], center = true);
            }
    }
}

// Complete Assembled 300mm 18650 FPV Flying Wing
module micro_flying_wing_18650() {
    union() {
        center_fuselage_18650();
        wing_half(is_right = true);
        wing_half(is_right = false);
    }
}

micro_flying_wing_18650();
