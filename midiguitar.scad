// translate([0, 0.5, pcbO]) import("midiguitarpcb.stl");

w = 50;
d = 51;
h = 30;
r = 2;
wedgeW = w + r;
wedgeD = 1.5;
wedgeH = 2;
pcbO = 5;

difference() {
	hull() {
		for (x = [0, w]) {
			for (y = [0, d]) {
				for (z = [0, h]) {
					translate([x, y, z]) rotate([0, 45, x > 0 != y > 0 ? -45 : 45]) cube(r, true);
				}
			}
		}
	}
	translate([0, wedgeD, 0]) cube([w + r, d - 2 * wedgeD, h]);
	translate([0, 0, pcbO]) cube([wedgeW, wedgeD, wedgeH]);
	translate([0, d - wedgeD, pcbO]) cube([wedgeW, wedgeD, wedgeH]);
	translate([0, d - 12.5, 8 + pcbO + wedgeH]) rotate([0, -90, 0]) cylinder(d = 12, h = r);
}
