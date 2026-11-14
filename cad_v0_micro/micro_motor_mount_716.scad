// =============================================================================
// Project Eagle-Eye v0 Micro: Standalone Snap-in 716 / 8520 Coreless Motor Nacelle
// Modular bracket with wing clip flange, cooling vents, and thrust angle alignment
// =============================================================================

$fn = 32;

motor_dia   = 7.4;  // 716 motor (7.0mm) + tolerance (or set 8.8 for 8520)
motor_len   = 16.8;
wall_thick  = 1.2;
flange_w    = 18.0;
flange_len  = 22.0;

module modular_motor_nacelle() {
    difference() {
        union() {
            // Cylindrical housing
            rotate([0, 90, 0])
                cylinder(r = (motor_dia / 2) + wall_thick, h = motor_len);

            // Wing mounting base flange
            translate([motor_len / 2, 0, -(motor_dia / 2) - 0.8])
                cube([flange_len, flange_w, 1.4], center = true);

            // Reinforcing aerodynamic gussets
            translate([motor_len * 0.45, 0, 0])
                hull() {
                    cube([10, flange_w - 4, 1.0], center = true);
                    translate([0, 0, (motor_dia / 2) + 0.5])
                        cube([6, 3, 1], center = true);
                }
        }

        // Motor bore hole
        translate([-1, 0, 0])
            rotate([0, 90, 0])
                cylinder(r = motor_dia / 2, h = motor_len + 3);

        // Rear shaft & pinion gear clearance
        translate([motor_len - 2, 0, 0])
            rotate([0, 90, 0])
                cylinder(r = 2.8, h = 5);

        // Top snap-fit expansion slot (enables snap-in friction fit without glue)
        translate([motor_len / 2, 0, (motor_dia / 2) + 0.5])
            cube([motor_len - 2, 1.8, 3.0], center = true);

        // Wing screw / pin alignment holes
        for (side = [-1, 1]) {
            translate([motor_len / 2, side * (flange_w / 2 - 2.5), -(motor_dia / 2) - 0.8])
                cylinder(r = 0.9, h = 4, center = true);
        }
    }
}

modular_motor_nacelle();
