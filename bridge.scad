// Global resolution
$fs = $preview ? 0.4 : 0.1;
$fa = $preview ? 0.5 :   1;  

n = 7;

length = 80;
width = 20;
height = 25;

gain = 3;

STEP_PERCENT_A = .30;
STEP_PERCENT_B = .30;
BASE_RESCALE = 2;
BEAM_RAD = 0.7;
BAL_RAD = 1.5;

JOINT_RAD = 0.9;
MARGIN = 3;

echo ("Print height", (MARGIN + 2*JOINT_RAD) * 5, "mm");

function cosh (x) = (exp (x) + exp(-x)) / 2;
function sinh (x) = (exp (x) - exp(-x)) / 2;

function norm_gain_cosh (x) = (-cosh(x * gain) + cosh(gain))/(cosh(gain) - 1);
function norm_gain_cosh_dx (x) = -gain * sinh(gain * x) / (cosh(gain) - 1);

// percent and plusminus norm
function p2pm (x) = 2 * ((x) - 0.5);
function pm2p (x) = (x + 1) / 2;

function swXZ (x) = [x.x, x.z];

// Cyl From To
module cft(from, to, rad) {
    let (
        vft = to - from,
        mag = sqrt(vft.x * vft.x + vft.y * vft.y + vft.z * vft.z),
        rotx = atan2(vft.x, vft.z),
        roty = atan2(vft.x, vft.y)
    ) {
       translate(from)
            rotate([rotx, 0, roty]) 
            cylinder(mag, rad, rad);
        /* echo("mag",mag); */
    }
}

module c3ft(from, to, rad) {
    let (
        vft = to - from,
        mag = sqrt(vft.x * vft.x + vft.y * vft.y + vft.z * vft.z),
        rotx = atan2(vft.x, vft.z),
        roty = atan2(vft.x, vft.y)
    ) translate(from)
        rotate([rotx, 0, roty])
        translate([-rad, -rad,0])
        cube([rad * 2, rad * 2, mag]);
}

module num (n,h=1) {
    color("teal") translate([0,0,-0.5])
        linear_extrude(height=h, convexity=4)
        text(str(n), 
            size=1,
            font="Consolas",
            halign="center",
            valign="center");
}

module arch (points, height) {
    for (i = [0:points -1]) 
        let (
            inc = length / points,
            center = length / 2,
            is = i + 0.5,
            i1 = is + 1,
            i2 = is - 1,

            x = p2pm(is / points),
            y = norm_gain_cosh(x),

            x1 = p2pm(i1 / points),
            y1 = norm_gain_cosh(x1),

            x2 = p2pm(i2 / points),
            y2 = norm_gain_cosh(x2),

            pt = [is * inc - center, 0, height * y],
            p1 = [i1 * inc - center, 0, height * y1],
            p2 = [i2 * inc - center, 0, height * y2],
            ll = [i  * inc - center, 0, 0],

            p1ptB = (p1 - pt)*STEP_PERCENT_B,
            p2ptB = (p2 - pt)*STEP_PERCENT_B,
            llptA = (ll - pt)*STEP_PERCENT_A,

            l_ssa1 = [
                [-llptA.x, llptA.z],
                swXZ(llptA),
            ],
            l_ssa2 = concat(
                i != points - 1 ? [[0,0], swXZ(p1ptB)] : [[0,0]],
                l_ssa1
            ),

            l_ssa3 = concat(
                l_ssa2,
                i != 0 ? [swXZ(p2ptB)] : []
            ),

            angle = atan(norm_gain_cosh_dx(x))
        ) {
            translate(pt + [0,MARGIN/2,0])
                rotate([90,0,0])
                cylinder(JOINT_RAD*2 + MARGIN, BAL_RAD, BAL_RAD, true);
            if(i != points-1)
                c3ft(
                    pt,
                    p1ptB + pt,
                    JOINT_RAD
                );
            if(i != 0)
                c3ft(
                    p2ptB + pt,
                    pt,
                    JOINT_RAD
                );
            c3ft(
                pt,
                llptA + pt,
                JOINT_RAD
            );
            c3ft(
                llptA + pt,
                pt,
                JOINT_RAD
            );

            translate(pt)
                rotate([90,0,0])
                linear_extrude(JOINT_RAD * 2, center=true)
                polygon(l_ssa3);

            translate(pt)
                rotate([0,-angle,0])
                translate([0, 0, BAL_RAD - 0.25])
                num(points + 1 + i);
        }
}

module cross_beams (points, height) {
    for (i = [0:points -1]) 
        let (
            inc = length / points,
            center = length / 2,
            is = i + 0.5,
            x = p2pm(is / points),
            y_val = norm_gain_cosh(x),
            base = [is * inc - center, 0, height*y_val]
        )
            translate(base)
            rotate([90,0,0])
            cylinder(width + JOINT_RAD + BEAM_RAD, BEAM_RAD,BEAM_RAD, true);
}

module beams (points, height) {
    for (i = [0:points - 2]) 
        let (
            inc = length / points,
            center = length / 2,

            is = i + 0.5,

            x0 = p2pm(is / points),
            y0 = norm_gain_cosh(x0),
            x1 = p2pm((is + 1) / points),
            y1 = norm_gain_cosh(x1)
        ) cft(
            [   is    * inc - center, 0, height * y0],
            [(is + 1) * inc - center, 0, height * y1],
            BEAM_RAD
        );
}

module baseline (points) {
    for ( i = [0 : points] )
        let (
            inc = length / points,
            center = length / 2,

            i1 = i + 0.5,
            i2 = i - 0.5,

            x1 = p2pm(i1 / points),
            y1 = norm_gain_cosh(x1),

            x2 = p2pm(i2 / points),
            y2 = norm_gain_cosh(x2),

            pt = [i * inc - center, 0, 0],
            p1 = [ i1   * inc - center, 0, height * y1],
            p2 = [ i2   * inc - center, 0, height * y2],

            p1ptB = (p1 - pt)*STEP_PERCENT_B,
            p2ptB = (p2 - pt)*STEP_PERCENT_B,

            l_ssa1 = [
                [JOINT_RAD * BASE_RESCALE, 0],
                [-JOINT_RAD * BASE_RESCALE, 0],
            ],
            l_ssa2 = concat(
                i != points ? [swXZ(p1ptB)] : [],
                l_ssa1
            ),
            l_ssa3 = concat(
                l_ssa2,
                i != 0 ? [swXZ(p2ptB)] : []
            )
        ) {
            translate(pt + [0,MARGIN/2,0])
                cube([
                    JOINT_RAD * 2 * BASE_RESCALE,
                    JOINT_RAD * 2 + MARGIN,
                    JOINT_RAD * 2
                ], true);

            if(i != points)
                c3ft(
                    pt,
                    p1ptB + pt,
                    JOINT_RAD
                );
            if(i != 0)
                c3ft(
                    p2ptB + pt,
                    pt,
                    JOINT_RAD
                );

            translate(pt)
                rotate([90,0,0])
                linear_extrude(JOINT_RAD * 2, center=true)
                polygon(l_ssa3);

            translate(pt + [0, 0, -JOINT_RAD + 0.3])
                num(i);
        }
}

module base_beams (points) {
    for ( i = [0 : points] )
        let (
            inc = length / points,
            center = length / 2
        )
            translate([i * inc - center, 0, 0])
            rotate([90,0,0])
            cylinder(width + JOINT_RAD + BEAM_RAD, BEAM_RAD, BEAM_RAD, true);
}

module supports (points, height) {
    for ( i = [0 : points -1] )
        let (
            inc = length / points,
            center = length / 2,
            is = i + 0.5,
            x = p2pm(is / points),
            y = norm_gain_cosh(x) * height
        ) cft(
            [i  * inc - center, 0, 0],
            [is * inc - center, 0,y],
            BEAM_RAD
        );
}


module ball_size () {
    difference() {
        union() {
            arch (n, height);
            baseline(n);
        }

        beams(n, height);
        supports(n, height);
        rotate([0,0,180]) 
            supports(n, height);
        rotate([0, 90, 0])
            cylinder(length, BEAM_RAD, BEAM_RAD, true);
    }
}
module balls () {
    if ($preview) translate([0,-10,0]) cylinder(JOINT_RAD * 2 + MARGIN,1,1);
    
    rotate([-90,0,0])
        translate([0, -width / 2 - JOINT_RAD, 0]) 
        difference() {
            translate([0, width/2,0]) rotate([180,180,0]) ball_size();
            base_beams(n);
            cross_beams(n, height);
        }
    
}
module side () {
    arch (n, height);
    beams(n, height);
    baseline(n);
    supports(n, height);
    rotate([0,0,180]) 
        supports(n, height);
    rotate([0, 90, 0])
        cylinder(length, BEAM_RAD, BEAM_RAD, true);
}

module vei () {
    color("red")
        translate([0,0,1.5*BEAM_RAD])
        cube([
            length + 2*JOINT_RAD * BASE_RESCALE,
            width - MARGIN * 2 - JOINT_RAD * 2,
            1
        ],center=true);

    for (i = [-1, 1])
        color("teal")
            translate([
                0,
                i * (width/2 - MARGIN/2 - JOINT_RAD),
                1* BEAM_RAD
            ])
            cube([
                length + 2 * JOINT_RAD * BASE_RESCALE,
                MARGIN,
                2
            ], center=true);


}

module main() {
    translate([0, width/2,0]) rotate([0,0,180]) { side(); }
    translate([0,-width/2,0]) { side(); }
    cross_beams(n, height);

    vei();

    base_beams(n);
}
main();
