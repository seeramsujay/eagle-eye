// =============================================================================
// Project Eagle-Eye v0 Micro: Ultra-Lightweight 3D-Printable Flying Wing
// Propulsion: Twin Coreless Motors (716 / 8520) - One in each wing half
// Steering: Differential Thrust (Yaw + Roll Coupling) + Reflexed Camber
// Wingspan: 210mm | Target AUW: < 35g (1S LiPo)
// =============================================================================

$fn = 32;

// Wing Geometry (mm)
span            = 210.0;
half_span       = span / 2.0;
root_chord      = 80.0;
tip_chord       = 42.0;
sweep_angle_deg = 26.0;
dihedral_deg    = 2.5;

// Coreless Motor Nacelle (716 / 8520 Coreless Brushed Motor)
motor_dia       = 7.4;   // 7.0mm motor + 0.4mm snap fit tolerance
motor_len       = 17.0;  // 16.5mm length
nacelle_y_pos   = 46.0;  // Distance from center to motor axis on each wing
prop_dia        = 55.0;  // 55mm micro propeller clearance

// Center Avionics & Battery Pod
pod_len         = 86.0;
pod_width       = 26.0;
pod_height      = 17.0;
battery_w       = 12.5;  // Standard 1S 300mAh LiPo width
battery_h       = 7.5;   // 1S LiPo thickness
battery_len     = 46.0;  // 1S LiPo length

// 2D Reflexed Airfoil Profile (Clean 2D manifold for linear_extrude)
module reflex_airfoil_2d(chord, thickness_ratio = 0.09) {
    t = chord * thickness_ratio;
    hull() {
        // Leading edge rounded entry
        translate([chord * 0.12, 0])
            circle(r = t * 0.45);
        // Maximum thickness crest (at 28% chord)
        translate([chord * 0.28, t * 0.12])
            circle(r = t * 0.50);
        // Reflexed trailing edge lift (S-camber upward curve at 85% chord)
        translate([chord * 0.85, t * 0.22])
            circle(r = t * 0.18);
        // Trailing edge
        translate([chord - 1.0, t * 0.28])
            square([1.5, 0.8], center = true);
    }
}

// Single Wing Half with Integrated Nacelle & Winglet
module wing_half(is_right = true) {
    dir = is_right ? 1 : -1;
    dx_tip = half_span * tan(sweep_angle_deg);
    dz_tip = half_span * sin(dihedral_deg);

    difference() {
        union() {
            // Wing Solid Loft between Root and Tip
            hull() {
                // Root airfoil section
                translate([0, dir * (pod_width / 2 - 2), 0])
                    rotate([90, 0, 0])
                        linear_extrude(height = 3.0, center = true)
                            reflex_airfoil_2d(root_chord);

                // Mid wing transition
                translate([dx_tip * 0.5, dir * (half_span * 0.5), dz_tip * 0.5])
                    rotate([90, 0, 0])
                        linear_extrude(height = 2.0, center = true)
                            reflex_airfoil_2d(root_chord * 0.72);

                // Tip airfoil section
                translate([dx_tip, dir * (half_span - 2.0), dz_tip])
                    rotate([90, 0, 0])
                        linear_extrude(height = 2.0, center = true)
                            reflex_airfoil_2d(tip_chord);
            }

            // Wingtip Vertical Stabilizer Fin (Winglet)
            translate([dx_tip + tip_chord * 0.35, dir * (half_span - 1.0), dz_tip + 12.0]) {
                rotate([0, -8, dir * 1.5]) // 8° rake, 1.5° toe-in
                    hull() {
                        cube([tip_chord * 0.85, 1.4, 2], center = true);
                        translate([tip_chord * 0.15, 0, 20.0])
                            cube([tip_chord * 0.35, 1.2, 2], center = true);
                    }
            }

            // Integrated Coreless Motor Nacelle Pod
            nacelle_x = (nacelle_y_pos / half_span) * dx_tip + 18.0;
            nacelle_z = (nacelle_y_pos / half_span) * dz_tip;

            translate([nacelle_x, dir * nacelle_y_pos, nacelle_z]) {
                hull() {
                    translate([0, 0, 0])
                        sphere(r = (motor_dia / 2) + 1.8);
                    translate([motor_len + 4, 0, 0])
                        cylinder(r = (motor_dia / 2) + 1.5, h = 2, center = true);
                }
            }
        }

        // Motor Nacelle Cavity & Wire Tunnel
        nacelle_x = (nacelle_y_pos / half_span) * dx_tip + 18.0;
        nacelle_z = (nacelle_y_pos / half_span) * dz_tip;

        translate([nacelle_x, dir * nacelle_y_pos, nacelle_z]) {
            // Coreless motor cylindrical bore
            translate([4, 0, 0])
                rotate([0, 90, 0])
                    cylinder(r = motor_dia / 2, h = motor_len + 10, center = true);

            // Motor rear shaft & prop boss exit
            translate([motor_len + 5, 0, 0])
                rotate([0, 90, 0])
                    cylinder(r = 3.2, h = 8, center = true);

            // Forward motor wire routing tunnel into wing body
            translate([-8, 0, 0])
                rotate([0, 90, dir * 45])
                    cylinder(r = 1.3, h = 22, center = true);

            // Cooling air vent slot on top of motor nacelle
            translate([motor_len * 0.4, 0, (motor_dia / 2) + 0.8])
                cube([8.0, 2.4, 4.0], center = true);
        }

        // Weight Reduction Core: Hollowed Wing Pockets
        translate([dx_tip * 0.45 + 16, dir * (half_span * 0.45), dz_tip * 0.45])
            scale([1, 1, 0.65])
                sphere(r = 14);

        translate([dx_tip * 0.75 + 14, dir * (half_span * 0.75), dz_tip * 0.75])
            scale([1, 1, 0.60])
                sphere(r = 10);
    }
}

// Center Fuselage Pod with Electronics & 1S Battery Bays
module center_fuselage() {
    difference() {
        // Outer Aerodynamic Shell
        hull() {
            // Nose cone
            translate([10, 0, 0])
                sphere(r = pod_width / 2);
            // Mid pod body
            translate([pod_len * 0.50, 0, 0])
                scale([1.4, 1.0, 0.85])
                    cylinder(r = pod_width / 2, h = pod_height, center = true);
            // Rear tail transition
            translate([pod_len - 4, 0, 2])
                scale([0.8, 0.6, 0.6])
                    cube([8, pod_width, pod_height], center = true);
        }

        // Upper Avionics Bay (Xiao / ESP32-C3 / MPU6050 / FETs)
        translate([pod_len * 0.48, 0, 3.0])
            cube([pod_len * 0.65, pod_width - 3.6, pod_height * 0.60], center = true);

        // Lower 1S LiPo Battery Bay (Quick Slide-in Underbelly Cartridge)
        translate([pod_len * 0.40, 0, -pod_height * 0.35]) {
            cube([battery_len, battery_w, battery_h + 1.0], center = true);
            // Ejection finger access slot
            translate([0, 0, -4])
                cube([battery_len - 12, 6.0, 6.0], center = true);
        }

        // Nose camera / FPV micro sensor port (10mm lens cutout)
        translate([4, 0, 1.5])
            rotate([0, 90, 0])
                cylinder(r = 4.5, h = 16, center = true);

        // Wing wire pass-through ports (Left & Right)
        for (side = [-1, 1]) {
            translate([pod_len * 0.42, side * (pod_width / 2), 1])
                rotate([90, 0, 0])
                    cylinder(r = 2.2, h = 8, center = true);
        }

        // Top cooling inlet scoop
        translate([26, 0, pod_height * 0.42])
            cube([12, 6, 6], center = true);
    }
}

// Assembled Complete 3D-Printable v0 Micro Flying Wing
module micro_flying_wing_v0() {
    union() {
        center_fuselage();
        wing_half(is_right = true);
        wing_half(is_right = false);
    }
}

micro_flying_wing_v0();
