w = 51;
d = 50;
h = 30;
r = 2 * sqrt(2);
wedgeW = 1.5;
wedgeD = d + r;
wedgeH = 2;
pcbO = 5;

difference() {
	hull() {
		for (x = [0, w]) {
			for (y = [0, d]) {
				for (z = [0, h]) {
					translate([x, y, z]) rotate([45, 45, 45]) cube(r, true);
				}
			}
		}
	}
	translate([wedgeW, 0, 0]) cube([w - 2 * wedgeW, d + r, h]);
	translate([0, 0, pcbO]) cube([wedgeW, wedgeD, wedgeH]);
	translate([w - wedgeW, 0, pcbO]) cube([wedgeW, wedgeD, wedgeH]);
	translate([12, 0, 7 + pcbO + wedgeH]) rotate([90, 0, 0]) cylinder(d = 12, h = r);
}
