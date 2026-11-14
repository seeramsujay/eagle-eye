// =============================================================================
// Project Eagle-Eye: High-Current Quick-Swap 3S 18650 Battery Cartridge
// Uses Bent Copper Strip Contacts (Zero High-Resistance Coiled Springs)
// Integrated Cantilever Snap-Lock Latch (Hot-Swap in 3 Seconds)
// =============================================================================

$fn = 32;

// 18650 Cell Dimensions (mm)
cell_dia       = 18.6;   // 18.3mm cell + 0.3mm tolerance
cell_len       = 65.6;   // 65.0mm cell + 0.6mm tolerance
cell_count     = 3;      // 3S1P (11.1V nominal)
wall_thick     = 1.8;
sled_width     = (cell_dia * cell_count) + (wall_thick * (cell_count + 1));
sled_len       = cell_len + 12.0; // Space for bent copper busbars & terminals
sled_height    = cell_dia * 0.78; // Deep cradle with positive cell retention

// Copper Strip Contact Dimensions (0.3mm x 8mm pure copper ribbon)
busbar_slot_w  = 8.5;
busbar_slot_t  = 1.2;

module cell_cradles() {
    difference() {
        // Solid cartridge frame block
        cube([sled_len, sled_width, sled_height]);

        // 3 Cylindrical Cell Bays
        for (i = [0 : cell_count - 1]) {
            y_pos = wall_thick + (cell_dia / 2) + i * (cell_dia + wall_thick);
            translate([sled_len / 2, y_pos, cell_dia / 2 + 1.2]) {
                rotate([0, 90, 0])
                    cylinder(r = cell_dia / 2, h = cell_len, center = true);

                // Top cell insertion & finger ejection slot
                translate([0, 0, cell_dia * 0.35])
                    cube([cell_len - 14, cell_dia * 0.82, cell_dia], center = true);
            }
        }

        // Copper Strip End Channels (Series Busbars)
        // Cell 0 (-) to Cell 1 (+) at Front
        translate([3.0, wall_thick + cell_dia, sled_height / 2])
            cube([busbar_slot_t, cell_dia + wall_thick, busbar_slot_w], center = true);

        // Cell 1 (-) to Cell 2 (+) at Rear
        translate([sled_len - 3.0, wall_thick + cell_dia * 2, sled_height / 2])
            cube([busbar_slot_t, cell_dia + wall_thick, busbar_slot_w], center = true);

        // Terminal Wire Exit Slots (XT30 / 14AWG Main Power Leads)
        // Positive Lead (+) at Front
        translate([2.5, wall_thick + cell_dia / 2, sled_height / 2])
            cylinder(r = 2.0, h = sled_height + 2, center = true);

        // Negative Lead (-) at Rear
        translate([sled_len - 2.5, wall_thick + (cell_count - 0.5) * (cell_dia + wall_thick), sled_height / 2])
            cylinder(r = 2.0, h = sled_height + 2, center = true);

        // 4-Pin JST-XH Balance Cable Routing Channel along bottom floor
        translate([sled_len / 2, sled_width / 2, 0.8])
            cube([sled_len - 8, 4.0, 1.6], center = true);

        // Weight reduction cutouts on underside
        for (i = [0 : cell_count - 1]) {
            y_pos = wall_thick + (cell_dia / 2) + i * (cell_dia + wall_thick);
            translate([sled_len / 2, y_pos, 0])
                cube([cell_len * 0.55, cell_dia * 0.5, 3], center = true);
        }
    }
}

// Flexible Cantilever Snap-Lock Latch (Front End)
module snap_lock_latch() {
    translate([-6.5, sled_width / 2 - 8, 0]) {
        difference() {
            union() {
                // Cantilever spring arm
                cube([7.0, 16.0, 2.4]);
                // Snap latch barb
                translate([0, 0, 2.4])
                    hull() {
                        cube([4.0, 16.0, 0.1]);
                        translate([-2.5, 0, 4.0])
                            cube([2.0, 16.0, 0.5]);
                    }
                // Finger pull tab
                translate([-5.0, 2.0, 3.5])
                    cube([5.0, 12.0, 3.0]);
            }
            // Finger grip serrations
            for (g = [0:3]) {
                translate([-4.5 + g * 1.0, 0, 6.0])
                    cube([0.5, 16.0, 1.0]);
            }
        }
    }
}

// Alignment Guide Rails for Fuselage Floor
module guide_rails() {
    for (y_side = [1.0, sled_width - 3.5]) {
        translate([6.0, y_side, -2.0])
            cube([sled_len - 12.0, 2.5, 2.0]);
    }
}

// Complete 3S 18650 Battery Cartridge Sled
module battery_sled() {
    union() {
        cell_cradles();
        snap_lock_latch();
        guide_rails();
    }
}

battery_sled();
