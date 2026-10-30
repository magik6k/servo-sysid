// SC15 pendulum test rig for actuator identification (BAM-style:
// a mass on an arm, swung by the servo against gravity).
//
// Parts (set `part` here, or: openscad -D 'part="stand"' -o stand.stl pendulum_rig.scad)
//   assembly  preview with a ghost servo, arm and mass; not for printing
//   stand     base + column + servo cradle; print upright, base on the bed
//   arm       pendulum arm; print flat, horn side down
//   heavy_arm stiffer arm for big masses (screw heads in counterbores);
//             print flat, horn side down
//   spool_hub bushing that centres a spool on the M8 bolt; print flange down
//   spool_clamp  disc that clamps the spool's front flange
//   retainer  plate that clamps the servo into the cradle
//   fit_test  6 mm slice of the cradle pocket; print first to check the fit
//
// Coordinates: origin on the servo axis, in the plane of the case's horn
// face. +Y points out of the horn face, +Z up; the horn axis is horizontal.
//
// Servo geometry is from Waveshare's SC15 drawings (shared/rig/vendor/sc15_cad):
// case 20.2 x 40.2 x 38.6 mm, horn axis centred across the width and
// 10.1 mm from one end, a coaxial idler boss 5.7 mm proud of the back,
// connectors on a broad side near the back. Horn disc: 4x M3 on a 14 mm
// bolt circle (opposite holes 14 mm apart).

part = "assembly";

/* [Servo] */
case_w = 20.2;          // X: across the horn face
case_l = 40.2;          // Z: along the horn face
case_d = 38.6;          // Y: depth behind the horn face
horn_from_end = 10.1;   // horn axis to the near (top) end of the case
boss_d = 11.5;          // idler boss on the back face, coaxial with the horn
boss_h = 5.7;
horn_top = 7.0;         // horn disc top above the case face; measure yours

/* [Horn disc] */
horn_disc_d = 20;
horn_pcd = 14;
horn_screw_clear = 3.4;  // M3
horn_center_clear = 9;   // centre screw head and boss

/* [Fit] */
clr = 0.3;               // pocket clearance per side
pilot = 2.8;             // M3 self-tapping into plastic; 4.0 for heat-set inserts

/* [Stand] */
axis_height = 165;       // horn axis above the table
side_wall = 5;
top_wall = 5;
bottom_wall = 6;
back_wall = 7;
column_setback = 6;      // column face behind the horn face: room for the mass bolt head
base_size = [120, 180, 6];
base_front = 70;         // base reaches this far in front of the horn face
base_hole = 5.5;
gusset_t = 6;
gusset_h = 60;

/* [Arm] */
hub_t = 4;               // print under the horn screw heads
hub_d = 28;
arm_w = 18;
arm_t = 8;
mass_radii = [50, 65, 80, 95, 110, 125];
mass_hole = 8.5;         // M8

/* [Heavy arm] */
heavy_t = 12;            // Screw heads sit in counterbores, hub_t above the horn
heavy_w = 26;
heavy_hub_d = 32;
counterbore_d = 6.5;     // M3 socket head

/* [Spool mount] */
spool_bore = 52;         // Spool centre hole; measure yours
spool_width = 50;        // Spool flange to flange; measure yours
spool_flange = 3;
spool_clamp_t = 4;

$fn = 64;
eps = 0.01;

// Derived cradle geometry.
pocket_x = case_w / 2 + clr;
pocket_top = horn_from_end + clr;
pocket_bottom = -(case_l - horn_from_end) - clr;
pocket_back = -case_d - clr;
cradle_x = pocket_x + side_wall;
cradle_top = pocket_top + top_wall;
cradle_bottom = pocket_bottom - bottom_wall;
cradle_back = pocket_back - back_wall;
cradle_front = -0.5;     // just behind the case face, so the retainer clamps the case
base_top = -axis_height + base_size.z;

retainer_screw_x = 10;
retainer_screw_z = (cradle_bottom + pocket_bottom) / 2;
retainer_t = 3;
retainer_overlap = 3;    // how far the retainer reaches over the case face

assert(max(mass_radii) + 25 < axis_height - base_size.z,
       "Arm plus mass would hit the base; raise axis_height or shorten the arm");

// 2D circle with a 45 degree point towards +Y, so horizontal holes print
// without support.
module teardrop2d(d) {
  circle(d = d);
  rotate(45) square(d / 2);
}

// Horizontal hole along Y from y0 to y1, centred at (x, z), point up.
module y_hole(d, x, z, y0, y1) {
  translate([x, y1, z]) rotate([90, 0, 0]) linear_extrude(y1 - y0) teardrop2d(d);
}

module cradle_block() {
  translate([-cradle_x, cradle_back, cradle_bottom])
    cube([2 * cradle_x, cradle_front - cradle_back, cradle_top - cradle_bottom]);
}

module pocket() {
  translate([-pocket_x, pocket_back, pocket_bottom])
    cube([2 * pocket_x, -pocket_back + 1, pocket_top - pocket_bottom]);
}

module stand() {
  column_front = -column_setback;

  difference() {
    union() {
      cradle_block();

      // Column from the base up to the cradle.
      translate([-cradle_x, cradle_back, base_top - eps])
        cube([2 * cradle_x, column_front - cradle_back, cradle_bottom - base_top + eps]);

      // 45 degree chamfer under the cradle's overhang past the column face.
      hull() {
        translate([-cradle_x, column_front - eps, cradle_bottom - (cradle_front - column_front)])
          cube([2 * cradle_x, eps, eps]);
        translate([-cradle_x, column_front - eps, cradle_bottom])
          cube([2 * cradle_x, cradle_front - column_front + eps, eps]);
      }

      translate([-base_size.x / 2, base_front - base_size.y, -axis_height]) cube(base_size);

      // Side gussets, centred on the column's depth.
      gusset_y = (cradle_back + column_front) / 2 - gusset_t / 2;
      for (s = [-1, 1])
        hull() {
          translate([s * cradle_x - eps / 2, gusset_y, base_top - eps])
            cube([eps, gusset_t, gusset_h]);
          translate([s * (base_size.x / 2 - 5) - eps / 2, gusset_y, base_top - eps])
            cube([eps, gusset_t, eps]);
        }

      // Back gusset.
      hull() {
        translate([-gusset_t / 2, cradle_back - eps, base_top - eps]) cube([gusset_t, eps, gusset_h]);
        translate([-gusset_t / 2, base_front - base_size.y + 5, base_top - eps]) cube([gusset_t, eps, eps]);
      }
    }

    pocket();

    // Idler boss through the back wall.
    y_hole(boss_d + 2, 0, 0, cradle_back - 1, pocket_back + eps);

    // Windows in both side walls for the connectors (near the back of a
    // broad face) and for routing cables.
    for (s = [-1, 1])
      translate([s > 0 ? pocket_x - eps : -cradle_x - 1, pocket_back + 1.5, pocket_bottom + 7])
        cube([side_wall + 1 + eps, 14, 26]);

    // Retainer screws into the bottom wall.
    for (s = [-1, 1])
      y_hole(pilot, s * retainer_screw_x, retainer_screw_z, cradle_front - 14, cradle_front + eps);

    // Base mounting holes (screws into a board, or use C-clamps on the base).
    for (sx = [-1, 1], y = [base_front - 10, base_front - base_size.y + 10])
      translate([sx * (base_size.x / 2 - 10), y, -axis_height - 1])
        cylinder(d = base_hole, h = base_size.z + 2);
  }
}

// Plate on the cradle's front face, from the cradle bottom up over the
// bottom edge of the case face. The cradle face sits 0.5 mm behind the
// case face, so tightening the screws clamps the case.
module retainer() {
  difference() {
    translate([-cradle_x, 0, cradle_bottom])
      cube([2 * cradle_x, retainer_t, -cradle_bottom + pocket_bottom + clr + retainer_overlap]);
    for (s = [-1, 1])
      translate([s * retainer_screw_x, -1, retainer_screw_z])
        rotate([-90, 0, 0]) cylinder(d = horn_screw_clear, h = retainer_t + 2);
  }
}

// Arm in its own frame: hub centre at the origin, horn side on z = 0,
// bar along +X. Thin (hub_t) under the screw heads, arm_t beyond them.
module arm() {
  r_end = max(mass_radii);
  ramp_start = hub_d / 2 - 1;
  ramp_end = hub_d / 2 + 4;

  difference() {
    union() {
      cylinder(d = hub_d, h = hub_t);
      translate([0, -arm_w / 2, 0]) cube([ramp_start + eps, arm_w, hub_t]);
      hull() {
        translate([ramp_start, -arm_w / 2, 0]) cube([eps, arm_w, hub_t]);
        translate([ramp_end, -arm_w / 2, 0]) cube([eps, arm_w, arm_t]);
      }
      translate([ramp_end, -arm_w / 2, 0]) cube([r_end - ramp_end, arm_w, arm_t]);
      translate([r_end, 0, 0]) cylinder(d = arm_w, h = arm_t);
    }

    translate([0, 0, -1]) cylinder(d = horn_center_clear, h = hub_t + 2);
    for (a = [45 : 90 : 315])
      rotate(a) translate([horn_pcd / 2, 0, -1]) cylinder(d = horn_screw_clear, h = hub_t + 2);
    for (r = mass_radii)
      translate([r, 0, -1]) cylinder(d = mass_hole, h = arm_t + 2);
  }
}

// Same frame as arm(). Full thickness right up to the hub, with the horn
// screw heads in counterbores so the same screws see hub_t of plastic.
module heavy_arm() {
  r_end = max(mass_radii);

  difference() {
    union() {
      cylinder(d = heavy_hub_d, h = heavy_t);
      translate([0, -heavy_w / 2, 0]) cube([r_end, heavy_w, heavy_t]);
      translate([r_end, 0, 0]) cylinder(d = heavy_w, h = heavy_t);
    }

    translate([0, 0, -1]) cylinder(d = horn_center_clear, h = heavy_t + 2);
    for (a = [45 : 90 : 315])
      rotate(a) translate([horn_pcd / 2, 0, 0]) {
        translate([0, 0, -1]) cylinder(d = horn_screw_clear, h = heavy_t + 2);
        translate([0, 0, hub_t]) cylinder(d = counterbore_d, h = heavy_t);
      }
    for (r = mass_radii)
      translate([r, 0, -1]) cylinder(d = mass_hole, h = heavy_t + 2);
  }
}

// Fills the spool's centre hole so it sits centred on the M8 bolt. Shorter
// than the spool, so the clamp disc presses on the spool, not the hub.
module spool_hub() {
  length = spool_flange + spool_width - 2;
  tube_d = spool_bore - 0.4;

  difference() {
    union() {
      cylinder(d = spool_bore + 16, h = spool_flange);
      difference() {
        cylinder(d = tube_d, h = length);
        translate([0, 0, -1]) cylinder(d = tube_d - 6, h = length + 2);
      }
      cylinder(d = 16, h = length);  // Bolt sleeve
      for (a = [0 : 90 : 270])
        rotate(a) translate([0, -1.5, 0]) cube([tube_d / 2 - 1, 3, length]);
    }
    translate([0, 0, -1]) cylinder(d = mass_hole, h = length + 2);
  }
}

module spool_clamp() {
  difference() {
    cylinder(d = spool_bore + 16, h = spool_clamp_t);
    translate([0, 0, -1]) cylinder(d = mass_hole, h = spool_clamp_t + 2);
  }
}

module fit_test() {
  linear_extrude(6) difference() {
    translate([-cradle_x, cradle_bottom]) square([2 * cradle_x, cradle_top - cradle_bottom]);
    translate([-pocket_x, pocket_bottom]) square([2 * pocket_x, pocket_top - pocket_bottom]);
  }
}

module ghost_servo() {
  color("DimGray", 0.6) {
    translate([-case_w / 2, -case_d, -(case_l - horn_from_end)]) cube([case_w, case_d, case_l]);
    translate([0, -case_d, 0]) rotate([90, 0, 0]) cylinder(d = boss_d, h = boss_h);
  }
  color("Silver", 0.8)
    translate([0, horn_top - 3, 0]) rotate([-90, 0, 0]) cylinder(d = horn_disc_d, h = 3);
}

// The arm hangs straight down, its horn side on the horn disc.
module placed_arm() {
  translate([0, horn_top, 0]) rotate([0, 90, 0]) rotate([-90, 0, 0]) children();
}

if (part == "stand") {
  translate([0, 0, axis_height]) stand();
} else if (part == "arm") {
  arm();
} else if (part == "heavy_arm") {
  heavy_arm();
} else if (part == "spool_hub") {
  spool_hub();
} else if (part == "spool_clamp") {
  spool_clamp();
} else if (part == "retainer") {
  // Print flat, case-side face on the bed.
  rotate([90, 0, 0]) translate([0, 0, -cradle_bottom]) retainer();
} else if (part == "fit_test") {
  fit_test();
} else {
  color("SteelBlue") stand();
  color("Orange") retainer();
  ghost_servo();
  color("SandyBrown") placed_arm() arm();
  // Example mass: 250 g of steel on an M8 bolt at the 95 mm hole.
  color("LightGray", 0.7)
    placed_arm() translate([95, 0, arm_t]) cylinder(d = 32, h = 40);
}
