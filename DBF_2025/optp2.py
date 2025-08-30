import aerosandbox as asb
import aerosandbox.numpy as np
from aerosandbox.tools import units

"""
Quick note before you go on replicating this code:
This is the code we landed on AFTER successfully testing a prototype. We developed a pretty accurate mass model, 
we already completed airfoil selection/ptimization, we already designed many components. 
If you're making your first optimization code, it SHOULD NOT look like this.
I typically start with some dumb weight estimates: Wing weight as a constant, fuselage weight as a constant, 
propulsion defined as a constant.
In fact, in your first pass, perhaps only payload weight, wing span, wing chord, velocity and AoA should be _real_ variables.

If you see any variables are frozen, they were probably forzen after we already froze some parts of the design, 
and chose to keep refining the rest.
"""

opti = asb.Opti()
airfoil = asb.Airfoil("sd7032")
symmetric_airfoil = asb.Airfoil("naca0010")

g= 9.81
rho=1.225
downwind_length = 1000*units.foot

#### WING SIZING

b = 6*units.foot ### doing this for wing weight model, I want to use int(b/spacing_between_ribs) as a constant  
 # for a first pass however, this really should be defined as an opti variable

# helper functions to make defining the wing easier.
def twist_func(x, twist):
    return -twist/(b/2)*x
def taper_func(x, rc, taper):
    return (taper*rc-rc)/b*x + rc
def constant_quarter_chord(root_chord, tip_chord):  
    return root_chord/4 - tip_chord/4

"""---- DENSITIES AND MATERIAL PROPERTIES"""
carbon_fiber_density = 1.75*1000 #kg/m^3
carbon_fiber_yield_strength = 600e6 #Pa
carbon_fiber_youngs_modulus = 230e9 #Pa
carbon_fiber_shear_modulus = 50e9 #Pa
carbon_fiber_layup_epoxy_factor = 2.2

fiberglass_density = 2.6*1000 #kg/m^3
fiberglass_youngs_modulus = 70e9 #Pa
fiberglass_shear_modulus = 30e9 #Pa
fiberglass_layup_epoxy_factor = 2.2

basswood_density = 320 #kg/m^3
basswood_youngs_modulus = 10e9 #Pa
basswood_shear_modulus = 3.8e9 #Pa
basswood_layup_epoxy_factor = 1.2
basswoord_shear_strength = 6.8e6 #Pa

plywood_density = 550 #kg/m^3

balsa_density = 160 #kg/m^3
balsa_youngs_modulus = 4e9 #Pa
balsa_shear_modulus = 1.5e9 #Pa
balsa_layup_epoxy_factor = 1.2
balsa_shear_strength = 2.1e6 #Pa

foam_density = 48 #kg/m^3

def fully_tapered(rc, tc):
    return (rc+tc)/2

#defining wing parameters
rc = opti.variable(init_guess=0.8, lower_bound=0.1, upper_bound=1.0, scale=0.1, category="rc")  # Root chord
tc = opti.variable(init_guess=0.1, lower_bound=0.1, upper_bound=1.0, scale=0.1, category="tc")  # Taper ratio
taper = tc/rc

# I'll note this, with aerosandbox (or any real MDO) I've found stability really really hard to control for. 
# Eventually, I have settled on this process: define initial guesses: washout = 2 
    # and dihedral = 3-5 is typically a good first guess.
# Once you have a converged design, export to AVL (or XFLR5, if you're nasty), and do some iterative design. 
# First time you do this: may take a few iterations. After a couple of practices, you'll just get what needs to change.
# I've already doen all this, we're happy with 5 degrees of washout and 2 degrees of dihedral: pilot approved :)
washout = 5
dihedral_angle = 2

# wing section definition:
wing_secs = [0,      # root
             0.04,   # fuselage-wing intersection    
             0.43,   # flap/aileron intersection
             1.82/2] # wingtip

control_surface_for_section = [[], 
                               [asb.ControlSurface(name="Flap", hinge_point=0.6, deflection=0)], 
                               [asb.ControlSurface(name="Aileron", symmetric=False, hinge_point=0.75, deflection=0)], 
                               []]

def taper_func(y, rc, taper):
    return (taper*rc-rc)/(b/2)*y + rc

def dihedral(y, dihedral):
    return y*np.tan(np.radians(dihedral))

def current_sec(i):
    y = wing_secs[i]
    chord = taper_func(y, rc, taper)
    x_disp = constant_quarter_chord(rc, chord)
    z_disp = dihedral(y, dihedral_angle)
    twist = twist_func(y, washout)
    control_surfaces = control_surface_for_section[i]
    return asb.WingXSec(xyz_le=[x_disp, y, z_disp], chord=chord, twist=twist, airfoil=airfoil, control_surfaces=control_surfaces)

wing = asb.Wing(name='Wing',
                symmetric=True,
                xsecs = [current_sec(i) for i in range(len(wing_secs))])


AR = wing.aspect_ratio()
S = wing.area() 
c_bar = wing.mean_geometric_chord()
# sweep = 
tail_dis = opti.variable(init_guess=1.25, lower_bound=1.0, upper_bound=2.5, scale=0.1, category="tail_dis", freeze=True)  # Distance from wing leading edge to horizontal tail

# at this point, we have already converged on a design and found some intersting results:
    # the sensitivity of the score with respect to wing area and taper is fairly low:
    # the 'optimum' was actually at S=1.2 m^2, but the score only imporved by a fraction of a percent.
    # since we woud rather make a smaller wing, that's what we settled on.
opti.subject_to([
    taper == 0.7,
    S == 0.9
    ])

##### VERTICAL TAIL SIZING
v_b = opti.variable(init_guess=0.4, lower_bound=0.1, upper_bound=1.8, scale=0.1, category="v_b", freeze=False)  # Span
v_c = opti.variable(init_guess=0.24, lower_bound=0.15, upper_bound=1, scale=0.1, category="v_c", freeze=False)  # Chord
vertical_tail = asb.Wing(name='Vertical Tail',
                        xsecs = [
                            asb.WingXSec(
                                xyz_le=[0, 0, 0],
                                chord=v_c,
                                twist=0,
                                airfoil=symmetric_airfoil,
                                control_surfaces=[asb.ControlSurface(name="Rudder", hinge_point=0.75, deflection=0)],
                            ),
                            asb.WingXSec(
                                xyz_le=[0, 0, v_b],
                                chord=v_c,
                                twist=0,
                                airfoil=symmetric_airfoil,
                            ),
                        ],
).translate([tail_dis, 0, 0])
Sv = vertical_tail.area(type='xz')


##### HORITZONTAL TAIL SIZING
h_b = opti.variable(init_guess=0.8, lower_bound=0.1, upper_bound=1.8, scale=0.1, category="h_b", freeze=False)  # Span
h_c = opti.variable(init_guess=0.24, lower_bound=0.15, upper_bound=1, scale=0.1, category="h_c", freeze=False)  # Chord


horizontal_tail = asb.Wing(name='Horizontal Tail',
                        symmetric=True,
                        xsecs = [
                            asb.WingXSec(
                                xyz_le=[0, 0, 0],
                                chord=h_c,
                                twist=0,
                                airfoil=symmetric_airfoil,
                                control_surfaces=[asb.ControlSurface(name="Elevator", hinge_point=0.75, deflection=0)],
                            ),
                            asb.WingXSec(
                                xyz_le=[0, h_b/2, 0],
                                chord=h_c,
                                twist=0,
                                airfoil=symmetric_airfoil,
                            ),
                            ]
).translate([tail_dis, 0, 0])

V_tail_volume = vertical_tail.area()*tail_dis/(S*b)
H_tail_volume = horizontal_tail.area()*tail_dis/(S*c_bar)

wings = [wing, horizontal_tail, vertical_tail]
# its really the same thing here as above. I've assigned a certain aspect ratio however, the tail is just redesigend in AVL.
opti.subject_to([h_c == v_c,
                H_tail_volume == 0.54,
                V_tail_volume == 0.07,
                horizontal_tail.aspect_ratio() < 4.0
                ])
Sh = horizontal_tail.area()


#### POD GEOMETRY DEFINITION
# the pod geometry is excessive and not very necessary, but the visuals look cool.
def elliptical_radius(x, length, val):
    x_over_c = x / length
    return val / 2 * (1 - (2 * x_over_c - 1) ** 2) ** 0.5

diameter = 0.0635 #m
pod_length = 0.203 #m
min_x = -0.05
max_x = min_x + pod_length
number_of_pods = 2
z_disp_pod = -0.15
xs = np.arange(0, pod_length, 0.01)

#plane
fuselages = []
Massprops = asb.MassProperties(mass=0)
# in order to estimate cg and moments of inertia, we will be modeling all components as point masses and adding them together.
# again, this is a marginally more developed model, you dont need this for a first prototype.

##### WING MASS DEFINITION
"""here, we are modeling the mass of a built up wing: here are the sections we consider:
    1. Ribs
    2. spar
    3. CF leading edge
    4. wing skin (arbitrarily small)
"""

#ribs
spacing_between_ribs = 0.15
rib_thickness = 0.0032 #m - this is just the thickness of the plywood we're using
rib_density = plywood_density # kg/m^3 --plywood density
rib_area = airfoil.area()/3 # alright, this was a total guess. Our ribs are hollowed out to save weight, 
                            # i guessed we removed around 2/3 of the surface, it turned out to be relatively true.

rib_epoxy_factor = 1.2 # how much epoxy do we include per rib mass.

wing_mass_per_rib = rib_thickness*rib_area*rib_density*rc*rib_epoxy_factor
centroidxtc, centroidytc, = airfoil.centroid()
num_ribs = int(b/spacing_between_ribs) #constant

wing_rib_mass_props = asb.MassProperties(mass=0)

#tapered
# to the MassProperties object above, we add each rib at a new x, y, z position as a function of sweep and 
for i in range(int(num_ribs/2)):
    chord = taper_func(spacing_between_ribs*i, rc, taper)
    rib_mass = rib_thickness*rib_density*chord*rib_area
    wing_rib_mass_props = wing_rib_mass_props + asb.MassProperties(mass = rib_mass, 
                                                                   x_cg = centroidxtc*chord + constant_quarter_chord(rc, taper_func(spacing_between_ribs*i, rc, taper)), 
                                                                   y_cg = spacing_between_ribs*i, z_cg = centroidytc*rc) + asb.MassProperties(mass = rib_mass, x_cg = centroidxtc*chord + constant_quarter_chord(rc, taper_func(spacing_between_ribs*i, rc, taper)), y_cg = -spacing_between_ribs*i, z_cg = centroidytc*rc)


wing_mass_props = wing_rib_mass_props

#monokote
wing_surface_area = wing.area(type='wetted') #m^2
tail_surface_area = vertical_tail.area(type="wetted") + horizontal_tail.area(type="wetted") #m^2

#assume surface material is monokote
monokote_density = 9/1000/0.092903#kg/m^2
monokote_mass = (wing_surface_area)*monokote_density
wing_monokote_mass_props = asb.MassProperties(mass = monokote_mass/2, x_cg = rc/2, y_cg = b/2, z_cg = 0) + asb.MassProperties(mass = monokote_mass/2, x_cg = rc/2, y_cg = -b/2, z_cg = 0)
wing_mass_props = wing_mass_props + wing_monokote_mass_props

#carbon leading edge
carbon_thickness = 0.06/1000 #m (0.06 mm)
leading_edge_area = airfoil.perimeter() * (rc+tc)/2 / 5 #5 is an estimate for the surface area of just the front of the wing
mass_leading_edge = carbon_fiber_density*leading_edge_area*carbon_thickness*b/2*carbon_fiber_layup_epoxy_factor
wing_carbon_leading_edge_mass_props = asb.MassProperties(mass = mass_leading_edge, x_cg = rc/5, y_cg = b/2, z_cg = 0) + asb.MassProperties(mass = mass_leading_edge, x_cg = rc/2, y_cg = -b/2, z_cg = 0)
wing_mass_props = wing_mass_props + wing_carbon_leading_edge_mass_props

#control surfaces
#flap
# we made flaps, I weighed them *shrug*
flap_mass = 0.075
flap_mass_props = asb.MassProperties(mass = flap_mass, x_cg = rc/2, y_cg = b/4, z_cg = 0) + asb.MassProperties(mass = flap_mass, x_cg = rc/2, y_cg = -b/4, z_cg = 0)

#aileron
aileron_mass = 0.053
aileron_mass_props = asb.MassProperties(mass = aileron_mass, x_cg = rc/2, y_cg = 3*b/4, z_cg = 0) + asb.MassProperties(mass = aileron_mass, x_cg = rc/2, y_cg = -3*b/4, z_cg = 0)

wing_mass_props = wing_mass_props + flap_mass_props + aileron_mass_props

Massprops = Massprops + wing_mass_props

#FOAM TAIL MASS ESTIMATE
## HORIZONTAL TAIL
ht_surface_area = horizontal_tail.area(type='wetted')
layup_thickness = 0.04/1000 #m (0.04mm)

fiberglass_mass = ht_surface_area*fiberglass_density*layup_thickness*fiberglass_layup_epoxy_factor
ht_fiberglass_massprops = asb.MassProperties(mass = fiberglass_mass, x_cg = tail_dis+v_c/2, y_cg = 0, z_cg = 0)

ht_foam_mass = horizontal_tail.xsecs[0].airfoil.area()*foam_density*h_c/5*h_b
ht_foam_massprops = asb.MassProperties(mass = ht_foam_mass, x_cg = tail_dis+v_c/2, y_cg = 0, z_cg = 0)

ht_carbon_mass = 2*0.05*h_b*layup_thickness*carbon_fiber_density*carbon_fiber_layup_epoxy_factor #0.05 is estimate for width, 0.0001 is estimate for thickness, 2 for both sides
ht_carbon_massprops = asb.MassProperties(mass = ht_carbon_mass, x_cg = tail_dis+v_c/2, y_cg = 0, z_cg = 0)

ht_mass_props = ht_fiberglass_massprops + ht_foam_massprops + ht_carbon_massprops
Massprops = Massprops + ht_mass_props

## VERTICAL TAIL
vt_surface_area = vertical_tail.area(type='wetted')

fiberglass_mass = vt_surface_area*fiberglass_density*layup_thickness*fiberglass_layup_epoxy_factor
vt_fiberglass_massprops = asb.MassProperties(mass = fiberglass_mass, x_cg = tail_dis+v_c/2, y_cg = 0, z_cg = 0)

vt_foam_mass = vertical_tail.xsecs[0].airfoil.area()*foam_density*v_c/5*v_b
vt_foam_massprops = asb.MassProperties(mass = vt_foam_mass, x_cg = tail_dis+v_c/2, y_cg = 0, z_cg = 0)

vt_carbon_mass = 2*0.05*v_b*layup_thickness*carbon_fiber_density*carbon_fiber_layup_epoxy_factor #0.05 is estimate for width, 0.0001 is estimate for thickness, 2 for both sides
vt_carbon_massprops = asb.MassProperties(mass = vt_carbon_mass, x_cg = tail_dis+v_c/2, y_cg = 0, z_cg = 0)

vt_mass_props = vt_fiberglass_massprops + vt_foam_massprops + vt_carbon_massprops
Massprops = Massprops + vt_mass_props

# landing gear
# landing gear mass assumed to be 500 g
landing_gear_mass = 0.5
landing_gear_mass_props = asb.MassProperties(mass = landing_gear_mass, x_cg = 0.0, y_cg = 0, z_cg = -0.2)

Massprops = Massprops + landing_gear_mass_props


#### PYLON POSITION DEFINITION
# pylon mass assumed to be 150 g
pylon_y_guess = 0.58
pylon_y_limit = b/2-0.15
# this is actually pretty cool, below, we will be defining the dimensions of the spar. 
# The spar is really heavy, to minimize mass, we want to be careful about where we attach pylons. 
# Turns out the optimum is pretty far out!
pylon_y = opti.variable(init_guess=pylon_y_guess, lower_bound=0.1, upper_bound=pylon_y_limit, scale=0.1, category="pylon_y", freeze=True)  # Pylon y location
pylon_mass = 0.130
pylon_mass_props = asb.MassProperties(mass = pylon_mass, x_cg = 0., y_cg = pylon_y, z_cg = 0) + asb.MassProperties(mass = pylon_mass, x_cg = 0., y_cg = -pylon_y, z_cg = 0)

Massprops = Massprops + pylon_mass_props


#### ELECTRONICS POSITION DEFINITION
#motor
# motor mass assumed, but location is a variable
motor_mass = 0.5
motor_x = opti.variable(init_guess=-0.25, lower_bound=-0.3, upper_bound=0, scale=0.1, category="motor_x")  # Motor x location
"""Why is motor location a variable?
Why for CG ofcourse! We can somewhat arbitraily place the motor, further forward to place cg optimally, 
but we'll find that's SUPER inefficient, because the boom is HEAVY.
"""
prop1_loc = [motor_x, 0, 0]
motor_mass_props = asb.MassProperties(mass = motor_mass, x_cg = prop1_loc[0], y_cg = prop1_loc[1], z_cg = prop1_loc[2])

electronics_mass_props = motor_mass_props

battery_mass = 0.750
battery_x = opti.variable(init_guess=-0.2, lower_bound=-0.3, upper_bound=0.3, scale=0.1, category="battery_x")  # Battery x location
battery_mass_props = asb.MassProperties(mass = battery_mass, x_cg = battery_x, y_cg = 0, z_cg = 0)
electronics_mass_props = electronics_mass_props + battery_mass_props

esc_mass = 0.1
esc_x = opti.variable(init_guess=-0.2, lower_bound=-0.3, upper_bound=0, scale=0.1, category="esc_x")  # ESC x location
esc_mass_props = asb.MassProperties(mass = esc_mass, x_cg = esc_x, y_cg = 0, z_cg = 0)
electronics_mass_props = electronics_mass_props + esc_mass_props

wing_servo_mass = 0.1
tail_servo_mass = 0.05
wing_servo_massprops = asb.MassProperties(mass=0)
tail_servo_massprops = asb.MassProperties(mass = tail_servo_mass, x_cg = tail_dis+h_c/2) + asb.MassProperties(mass = tail_servo_mass, x_cg=tail_dis+h_c/2, z_cg=v_b/2)
for i in range(2):
    loc = b/4*i+b/8
    wing_servo_massprops = wing_servo_massprops + asb.MassProperties(mass = wing_servo_mass, x_cg = loc) + asb.MassProperties(mass = wing_servo_mass, x_cg=-loc)
electronics_mass_props = electronics_mass_props + wing_servo_massprops + tail_servo_massprops

wire_length = (b/2+b/4 + # wing servo wires
               tail_dis * 2 + # tail servo wires
               motor_x*2) #wires to motor
wire_mass_per_m = 0.012
wire_mass = wire_length*wire_mass_per_m
wire_massprops = asb.MassProperties(mass = wire_mass, x_cg = rc) #rc is estimate
electronics_mass_props = electronics_mass_props + wire_massprops

Massprops = Massprops + electronics_mass_props

opti.subject_to([
    motor_x+0.1 < esc_x,
    motor_x+0.1 < battery_x,
])
fuselage_mass = 0.27 #fuselage mass replaced with final fuselage mass
fuselage_mass_props = asb.MassProperties(mass = fuselage_mass, x_cg = rc/4, y_cg = 0, z_cg = 0)
Massprops = Massprops + fuselage_mass_props

##### BOOM SIZING
#carbon fiber
material_density = carbon_fiber_density #kg/m^3
material_youngs_modulus = carbon_fiber_youngs_modulus*0.4 # 40% is just an estimate given 45-45 layup, turns out, it's fairly true.

""" 
Defining the dimensions of the boom as constrained by 2 load cases:
    1. torsional load from max sideslip
    2. bending load from 1.0 CL pull up maneuver.
    
In general, we've found the first to be more constraining.
"""
material_torsional_modulus = material_youngs_modulus/(2*(1+0.7)) #Pa #.7 assumes CF poisson ratio
boom_outer_h = opti.variable(init_guess=0.028, lower_bound=0.01, upper_bound=0.1, scale=0.01, category="boom_outer_diameter", freeze = True)  # Outer diameter of the boom
boom_inner_h = opti.variable(init_guess=0.0225, lower_bound=0.001, upper_bound=0.1, scale=0.01, category="boom_inner_diameter", freeze = True)  # Inner diameter of the boom
boom_length = tail_dis + v_c - motor_x#m
boom_mass = material_density*(boom_outer_h**2-boom_inner_h**2)*boom_length * 0.75 # adding factor of 0.75 to correct based on the mass of the purchased boom.
x_cg_boom = (tail_dis + v_c + motor_x)/2
boom_mass_props = asb.MassProperties(mass = boom_mass, x_cg = x_cg_boom, y_cg = 0, z_cg = 0)

Massprops = Massprops + boom_mass_props
opti.subject_to(boom_length == 1.7) # frozen after we already purchased the boom

assumed_tail_CLmax = 1.0
assumed_max_velocity_for_tail = 45 # cruise speed is about ~30 im setting the max displacement at and 10 degrees aoa to be 4 cm
force_on_tail = assumed_tail_CLmax*assumed_max_velocity_for_tail**2*Sh*rho/2 

boom_I = 1/12*(boom_outer_h**4-boom_inner_h**4)
boom_J = 2/3*(boom_outer_h**4-boom_inner_h**4)
boom_minor_length = boom_length - rc + motor_x # boom minor length just measures boom length from the end of the wing.

#defining boom deflection
boom_deflection = force_on_tail*boom_minor_length**3/(3*material_youngs_modulus*boom_I)

# defining max compressive stress.
compressive_stress = force_on_tail*boom_minor_length/(boom_outer_h**2-boom_inner_h**2)

#defining torsional force and stress
tail_torsional_force = v_b/2 * 1/2*rho*assumed_max_velocity_for_tail**2*Sv*assumed_tail_CLmax
torsional_shear_stress_boom = tail_torsional_force*boom_outer_h/2/(boom_J)
material_max_shear_stress = 207e6 #Pa #aluminum


tail_deflection_limit = b*0.02 # our stiffness limit is 2% deflection at a max load case

"""This is actually how you do some boom sizing, constrain dimensions, (always make sure you enforce positive dimensions)
Then constrain the maximum stresses to be under the material yield stress and material max shear stress.
Constrain the deflection to be within acceptable limits, in our case, 2% is a pretty good bet."""
# opti.subject_to([boom_deflection < tail_deflection_limit,
#                 boom_outer_h > boom_inner_h,
#                 # boom_outer_h-boom_inner_h > 0.002,
#                 compressive_stress < material_yield_strength,
#                 torsional_shear_stress_boom < material_torsional_modulus,
#                 ])


#tail boom
tail_boom = asb.Fuselage('Tail Boom',
                        xsecs = [
                            asb.FuselageXSec(
                                xyz_c=[motor_x, 0, 0],
                                xyz_normal=[-1, 0, 0],
                                radius = boom_outer_h/2,
                                shape = 10,
                            ),
                            asb.FuselageXSec(
                                xyz_c=[tail_dis+v_c, 0, 0],
                                xyz_normal=[-1, 0, 0],
                                radius = boom_outer_h/2,
                                shape = 10,
                            ),
                        ]
)
fuselages += [tail_boom]


#### PAYLOAD MASS DEFINITION
"""
Here, we're defining the payload parameters. We want to set both the payload mass and the position. For now, we're assuming the cg is at quarter-chord
"""
payload_mass = opti.variable(init_guess=10, lower_bound=0.1, upper_bound=25, scale=0.1, freeze = True, category="payload_mass")  # Payload mass
# opti.subject_to(payload_mass < 10*1.2)
payload_cg = opti.variable(init_guess=0., lower_bound=-0.2, upper_bound=pod_length, scale=0.1, category="payload_cg", freeze=True)  # Payload cg
payload_mass_props = asb.MassProperties(mass = payload_mass/2, x_cg = payload_cg+pod_length/2, y_cg = pylon_y, z_cg =z_disp_pod) + asb.MassProperties(mass = payload_mass/2, x_cg = payload_cg+pod_length/2, y_cg = -pylon_y, z_cg = z_disp_pod)

##### SPAR SIZING
# wing spar is sized by applying a distributed maximum load factor, and restricting the displacement at the wing tip
airfoil_thickness = airfoil.max_thickness()
tip_thickness = airfoil_thickness*tc

##### SANDWICH SPAR SIZING
# Add constraints
positive_limit_load_factor = 10
negative_limit_load_factor = 4

# Define core material properties
core_density = balsa_density*2
core_shear_strength = balsa_shear_strength

# Define cap material properties
cap_material_density = carbon_fiber_density
cap_material_yield_strength = carbon_fiber_yield_strength
cap_material_youngs_modulus = carbon_fiber_youngs_modulus

spar_width = opti.variable(
    init_guess=22.5/1000,
    lower_bound= 4/1000,
    upper_bound=25/1000,
    scale=0.1,
    category="spar_width",
    freeze=True
)  # Width of the spar caps and core

spar_cap_thickness = opti.variable(
    init_guess=2.6/1000,
    lower_bound= 0.8/1000,
    upper_bound=5/1000,
    scale=0.01,
    category="spar_cap_thickness",
    freeze=True
)  # Thickness of the outer skins

# asb.Airfoil().local_thickness()
core_thickness_root = wing.xsecs[0].airfoil.local_thickness(x_over_c = 0.25)*rc-spar_cap_thickness*2

core_thickness_tip = wing.xsecs[-1].airfoil.local_thickness(x_over_c = 0.25)*tc-spar_cap_thickness*2

# Compute spanwise core thickness variation (linear taper)
def core_thickness(y):
    eta = 2 * y / b  # Non-dimensional spanwise location (eta = 0 at root, 1 at tip)
    return core_thickness_root * (1 - eta) + core_thickness_tip * eta

# Mass properties

spar_carbon_mass_no_epoxy = 2 * spar_width * spar_cap_thickness * b * cap_material_density/6 # 2x because we have a top and bottom cap. /6 factor to align with measured masses of spar cap.
spar_carbon_mass_with_epoxy = spar_carbon_mass_no_epoxy * carbon_fiber_layup_epoxy_factor # add epoxy mass. 
spar_carbon_massprops_no_epoxy = asb.MassProperties(mass = spar_carbon_mass_no_epoxy/2, x_cg = rc/4, y_cg = b/4, z_cg = 0) + asb.MassProperties(mass = spar_carbon_mass_no_epoxy/2, x_cg = rc/4, y_cg = -b/4, z_cg = 0)
spar_carbon_massprops_epoxy = asb.MassProperties(mass = spar_carbon_mass_with_epoxy/2, x_cg = rc/4, y_cg = b/4, z_cg = 0) + asb.MassProperties(mass = spar_carbon_mass_with_epoxy/2, x_cg = rc/4, y_cg = -b/4, z_cg = 0)

spar_core_mass = spar_width * (core_thickness_root + core_thickness_tip) / 2 * b * core_density / 4# /2 to fit measurements
spar_core_massprops = asb.MassProperties(mass = spar_core_mass/2, x_cg = rc/4, y_cg = b/4, z_cg = 0) + asb.MassProperties(mass = spar_core_mass/2, x_cg = rc/4, y_cg = -b/4, z_cg = 0)

spar_fiberglass_mass_no_epoxy = (2 * (core_thickness_root+core_thickness_tip)/2 * b)* layup_thickness * fiberglass_density  # /2 to fit measurements
spar_fiberglass_mass_with_epoxy = spar_fiberglass_mass_no_epoxy * fiberglass_layup_epoxy_factor
spar_fiberglass_massprops_no_epoxy = asb.MassProperties(mass = spar_fiberglass_mass_no_epoxy/2, x_cg = rc/4, y_cg = b/4, z_cg = 0) + asb.MassProperties(mass = spar_fiberglass_mass_no_epoxy/2, x_cg = rc/4, y_cg = -b/4, z_cg = 0)
spar_fiberglass_massprops_epoxy = asb.MassProperties(mass = spar_fiberglass_mass_with_epoxy/2, x_cg = rc/4, y_cg = b/4, z_cg = 0) + asb.MassProperties(mass = spar_fiberglass_mass_with_epoxy/2, x_cg = rc/4, y_cg = -b/4, z_cg = 0)

spar_mass_with_epoxy = spar_carbon_mass_with_epoxy + spar_core_mass + spar_fiberglass_mass_with_epoxy
spar_mass_no_epoxy = spar_carbon_mass_no_epoxy + spar_core_mass + spar_fiberglass_mass_no_epoxy
spar_epoxy_mass = spar_mass_with_epoxy - spar_mass_no_epoxy

spar_mass = spar_carbon_mass_with_epoxy

# Bending moment of inertia (Equation 16) - Unified Notes. LMK if you need a link.
spar_I = (
    spar_width * spar_cap_thickness * (core_thickness_root + spar_cap_thickness)**2
)  # Only outer skins (caps) contribute significantly

#positive load factor
# Distributed load (positive or negative z-direction)
# Quick note, typically, the load distribution is ~elliptical. However, this is more complicated. 
# We will size with the conservative estimate, which is uniform (bending moment is higher in this case)
positive_distributed_load_per_length = positive_limit_load_factor * (Massprops.mass + spar_mass) / b * g  # Uniform load

# Point load (positive or negative z-direction)
# Payload point load
point_load_positive = payload_mass/2 * g * positive_limit_load_factor

# negative load factor 
# Distributed load (positive or negative z-direction)
negative_distributed_load_per_length = negative_limit_load_factor * (Massprops.mass + spar_mass) / b * g  # Uniform load

# Point load (positive or negative z-direction)
point_load_negative = payload_mass/2 * g * negative_limit_load_factor

# Bending moments for each load case
# 1. Distributed + Point Load in Positive Z-Direction
moment_positive = (
    positive_distributed_load_per_length * b**2 / 2 -  # Max moment due to distributed load (root)
    point_load_positive * (b - pylon_y)  # Additional moment due to point load
)

# 2. Distributed + Point Load in Negative Z-Direction
moment_negative = (
    -negative_distributed_load_per_length * b**2 / 2 +  # Max moment due to distributed load (root)
    -point_load_negative * (b - pylon_y)  # Additional moment due to point load
)

# Tip deflections for each load case (Equation 19)
deflection_positive = moment_positive * (b / 2)**2 / (2 * cap_material_youngs_modulus * spar_I)
deflection_negative = moment_negative * (b / 2)**2 / (2 * cap_material_youngs_modulus * spar_I)

# Shear stress in the core
shear_force_positive = positive_distributed_load_per_length * b / 2 - point_load_positive  # Shear force at root
shear_force_negative = -negative_distributed_load_per_length * b / 2 - point_load_negative  # Shear force at root
shear_stress_core_positive = shear_force_positive / (spar_width * core_thickness_root)
shear_stress_core_negative = shear_force_negative / (spar_width * core_thickness_root)

# Bending stress in spar caps
bending_stress_positive = moment_positive * (core_thickness_root + spar_cap_thickness) / (2 * spar_I)
bending_stress_negative = moment_negative * (core_thickness_root + spar_cap_thickness) / (2 * spar_I)

tip_deflection_limit = 1.82*0.02 # Maximum allowable deflection (stiffness constraint)
opti.subject_to([
    deflection_positive < tip_deflection_limit,  # Tip deflection for positive load case
    -deflection_negative < tip_deflection_limit,  # Tip deflection for negative load case
    shear_stress_core_positive < core_shear_strength,  # Core shear strength constraint (positive load case)
    shear_stress_core_positive > -core_shear_strength,
    shear_stress_core_negative < core_shear_strength,  # Core shear strength constraint (negative load case)
    shear_stress_core_negative > -core_shear_strength,
    bending_stress_positive < cap_material_yield_strength,  # Cap strength constraint (positive load case)
    bending_stress_positive > -cap_material_yield_strength,
    bending_stress_negative < cap_material_yield_strength,  # Cap strength constraint (negative load case)
    bending_stress_negative > -cap_material_yield_strength,
    core_thickness_root > core_thickness_tip,  # Ensure taper direction is correct
])

# Update mass properties
x_cg_spar = rc * 0.25
wing_spar_mass_props = spar_carbon_massprops_epoxy + spar_fiberglass_massprops_epoxy + spar_core_massprops
wing_mass_props = wing_mass_props + wing_spar_mass_props
Massprops = Massprops + wing_spar_mass_props


# sandwich spar
empty_mass_props = Massprops #with pylons, without weight

total_mass_props = empty_mass_props + payload_mass_props # full payload
total_mass = total_mass_props.mass



""" ------ AERODYNAMICS --------"""
#### AERODYNAMIC ANALYSIS
# aerodynamic definition
pod1 = asb.Fuselage('Pod1',
                    xsecs=[
                        asb.FuselageXSec(
                            xyz_c=[x, 0, 0],
                            xyz_normal=[-1, 0, 0],
                            radius=elliptical_radius(x, pod_length, diameter),
                            shape=2,
                        ) for x in xs
                    ],
                ).translate([payload_cg, pylon_y, z_disp_pod])
pod2 = asb.Fuselage('Pod2',
                    xsecs=[
                        asb.FuselageXSec(
                            xyz_c=[x, 0, 0],
                            xyz_normal=[-1, 0, 0],
                            radius=elliptical_radius(x, pod_length, diameter),
                            shape=2,
                        ) for x in xs
                    ],
                ).translate([payload_cg, -pylon_y, z_disp_pod])

motor = asb.Fuselage('Motor', #purely for drawing
                     xsecs = [
                         asb.FuselageXSec(
                             xyz_c=[motor_x, 0, 0],
                             xyz_normal=[-1, 0, 0],
                             radius = 0.07,
                             shape = 2
                         ),
                            asb.FuselageXSec(
                                xyz_c=[motor_x+0.06, 0, 0],
                                xyz_normal=[-1, 0, 0],
                                radius = 0.07,
                                shape = 2
                            )
                     ])

fuselages += [pod1, pod2]

pod_volume = pod1.volume()
total_fuel_volume = pod_volume*number_of_pods

#1 prop
prop_diameter = opti.variable(init_guess=20, lower_bound=6, upper_bound=24, scale=1, category="prop_diameter") #units inch
normal = [-1, 0, 0]
num_motor = 1
# prop_efficiency = 0.65

prop1 = asb.Propulsor('Motor',
                    xyz_c=prop1_loc,
                    radius=prop_diameter*units.inch/2,
                    xyz_normal = normal,
                    )
props = [prop1]

plane_m2 = asb.Airplane(name='Plane full',
                    xyz_ref= [total_mass_props.x_cg, 0, total_mass_props.z_cg],
                    wings = wings,
                    propulsors=props,
                    fuselages=fuselages,
)
plane_m1_m3 = asb.Airplane(name='Plane empty',
                    xyz_ref= [empty_mass_props.x_cg, 0, empty_mass_props.z_cg],
                    wings = wings,
                    propulsors=props,
                    fuselages=fuselages,
)

velocity_full = opti.variable(init_guess=31.28, lower_bound=1, upper_bound=50, scale=5, category="velocity_full", freeze=False)  # Velocity


"""Aerosandbox has some basic drag modeling, we're largely using an offshoot of DATCOM.
In my experience, it underestimates a lot of drag, so we want to bump up some numbers, especially for all sources that are not accounted for.
"""
def wheel(velocity, diameter, CD):
    S = np.pi*(diameter/2)**2
    drag = 0.5*rho*velocity**2*CD*S
    return drag

wheel_diameter = 77.8/1000 #m
wheel_S = np.pi*(wheel_diameter/2)**2
CD_landing_gear = 0.52 # Could we get this with CFD, sure, but honestly, we don't need that precision, this is directly out of Hoerner.
drag_landing_gear = 0.5*rho*velocity_full**2*CD_landing_gear*wheel_S

    #fuel pods
    #assuming 0.5 L soda bottle: 
def fuel_pods(velocity, diameter, num_pods=2):
    CD = 0.26*num_pods
    S = np.pi*(diameter/2)**2
    drag = 0.5*rho*velocity**2*CD*S
    return drag

diameter = 0.0635 #m
number_of_pods = 2
# length = 0.203 #m
CD_pod = 0.26*number_of_pods # Hoerner 13-16
S_pod = np.pi*(diameter/2)**2
drag_pods = 0.5*rho*velocity_full**2*CD_pod*S_pod

"""Brief not here about propulsion sizing. This is the worst possible way to do this. I cannot stress this enough, do not use whatever numbers this comes up with. 
Why? well, in this case, we're sizing the entire propulsion based on a particular operating point at cruise. 
Guess what? You'll have so many more operating regimes during your flight. 
This gives a VERY GENERAL ESTIMATE of power requirements. Really, you want to be ensuring your propulsion performs well over the entire envelope."""

propulsion_efficiency = 0.65 # should be set by dynamic propulsion testing, we always overestimate our efficiency. It usually ends up being 60-70%
prop_pitch = opti.variable(init_guess=10, lower_bound=5, upper_bound=24, scale=1, category="prop_pitch", freeze=False)  # Pitch
Kv = opti.variable(init_guess=540, lower_bound=400, upper_bound=2000, scale=100, category="Kv", freeze=False)  # KV
voltage = 29.6 # 8S battery
Pmax = opti.variable(init_guess=1760, lower_bound=0, upper_bound=2000, scale=100, category="Pmax", freeze=False)  # Power
rpm_max = opti.variable(init_guess=20000, lower_bound=5000, upper_bound=24000, scale=1000, category="rpm_max", freeze=False)  # RPM

max_thrust = 4.392399e-8*rpm_max*prop_diameter**3.5/(prop_pitch**0.5)*4.23333e-4*rpm_max*prop_pitch
I = Pmax/(voltage*propulsion_efficiency)
total_energy = 240000 # converted from mAh

disk_loading = max_thrust/(np.pi*(prop_diameter*units.inch/2)**2)
tip_speed = rpm_max/60*(2*np.pi*(prop_diameter*units.inch/2)) #m/s

opti.subject_to([
    I<100,
    tip_speed/340 < 0.95
    # disk_loading < 500
    ])


###### MISSION ANALYSIS
turn_load_factor = 6 # estimate

#mission 2
thrust_full = opti.variable(init_guess=30, lower_bound=0, upper_bound=100, scale=10, category="thrust_full")
alpha_full = opti.variable(init_guess=0, lower_bound=-10, upper_bound=10, scale=5, category="alpha_full")  # Angle of attack
velocity_full = opti.variable(init_guess=31.28, lower_bound=1, upper_bound=50, scale=5, category="velocity_full", freeze=False)  # Velocity
op_point_full = asb.OperatingPoint(velocity=velocity_full, alpha=alpha_full)
aero_full = asb.AeroBuildup(plane_m2, op_point_full).run_with_stability_derivatives()

prop_rpm_full = opti.variable(init_guess=18000, lower_bound=0, upper_bound=24000, scale=1000, category="prop_rpm_full") #1/s
throttle_full = opti.variable(init_guess=1.0, lower_bound=0, upper_bound=1, scale=0.1, category="throttle_full", freeze=False)  # Throttle
m2_lap_length = opti.variable(init_guess=1000*units.foot*1.5, scale=10, category="m2_lap_length")  # Lap length

# m2_takeoff_energy = opti.variable(init_guess=0, lower_bound=0, upper_bound=total_energy, scale=1, category="m2_takeoff_energy")  # Takeoff energy
# m2_cruise_energy = opti.variable(init_guess=0, lower_bound=0, upper_bound=total_energy, scale=1, category="m2_cruise_energy")  # Cruise energy

# ballast_full = opti.variable(init_guess=0, lower_bound=0, upper_bound=10, scale=1, category="ballast_full")  # Ballast mass
# ballast_x_full = opti.variable(init_guess=-0.1, lower_bound=-0.2, upper_bound=0.5, scale=0.1, category="ballast_x_full")  # Ballast cg
# total_mass_props = total_mass_props + asb.MassProperties(mass = ballast_full, x_cg = ballast_x_full, y_cg = 0, z_cg = 0)

tip_velocity_full = prop_rpm_full*(prop_diameter*units.inch*np.pi/60/2)

"""Okay, here we get to our favorite equation. This equation is derived analtically by ___. 
Through extensive testing, it's actually REALLY accurate."""
opti.subject_to([
    thrust_full == 4.392399e-8*prop_rpm_full*prop_diameter**3.5/(prop_pitch**0.5)*(4.23333e-4*prop_rpm_full*prop_pitch-velocity_full),
    prop_rpm_full == Kv*throttle_full*voltage,  
    thrust_full < Pmax/velocity_full,
    tip_velocity_full < 343,
    throttle_full == 0.85
])

# alright, here we're getting REALLY hand-wavy. Our drag was too low given other analyses, mltiplying aeorbuidup CD by 2 matched other estimates.
# This ends up happening when we use many analyses of different fidelities :/.
drag_full = aero_full['D']*2 + fuel_pods(velocity_full, diameter, num_pods=2) + wheel(velocity_full, wheel_diameter, CD_landing_gear)
CD_full = drag_full/(0.5*rho*velocity_full**2*S)

opti.subject_to([
    drag_full == thrust_full,
    aero_full['L'] == total_mass_props.mass*g,
])

m2_vstall = np.sqrt(2*total_mass_props.mass*g/(rho*S*1.6))
avg_speed_takeoff_m2 = 0.9*m2_vstall
m2_takeoff_max_thrust = 4.392399e-8*prop_rpm_full*prop_diameter**3.5/(prop_pitch**0.5)*(4.23333e-4*prop_rpm_full*prop_pitch-avg_speed_takeoff_m2)
m2_takeoff_drag = 0.5*rho*avg_speed_takeoff_m2**2*CD_full*S
m2_takeoff_accel = (m2_takeoff_max_thrust-m2_takeoff_drag)/total_mass_props.mass
m2_takeoff_distance = m2_vstall**2/(2*m2_takeoff_accel)
m2_takeoff_energy = m2_takeoff_distance*m2_takeoff_max_thrust*propulsion_efficiency
m2_cruise_energy = thrust_full*m2_lap_length*4*propulsion_efficiency
m2_takeoff_time = m2_takeoff_distance/avg_speed_takeoff_m2
m2_energy = m2_takeoff_energy + m2_cruise_energy

opti.subject_to(
    m2_energy < total_energy
)

m2_turn_velocity = velocity_full*1.2
m2_turning_radius = m2_turn_velocity**2/(g*np.sqrt((turn_load_factor)**2-1))
m2_distance_for_turn = 2*np.pi*m2_turning_radius*2 #in 1 lap, 2 half turns, + 1 full turn
m2_time = m2_takeoff_time + m2_lap_length*3/velocity_full
opti.subject_to(m2_lap_length == downwind_length*2 + m2_distance_for_turn*2)
m2_time = m2_lap_length*3/velocity_full

mission2_score = payload_mass_props.mass/m2_time

opti.subject_to(m2_time < 5*60)

#mission 3
m3_voltage = 22.2 # 6S battery
thrust_empty = opti.variable(init_guess=18, lower_bound=0, upper_bound=100, scale=10, category="thrust_empty")
alpha_empty = opti.variable(init_guess=3, lower_bound=-10, upper_bound=10, scale=5, category="alpha_empty")  # Angle of attack
velocity_empty = opti.variable(init_guess=30, lower_bound=1, upper_bound=50, scale=5, freeze=False, category="velocity_empty")  # Velocity
op_point_empty = asb.OperatingPoint(velocity=velocity_empty, alpha=alpha_empty)
number_of_laps_empty = opti.variable(init_guess=7, lower_bound=1, upper_bound=20, scale=1, freeze=False, category="number_of_laps")
aero_empty = asb.AeroBuildup(plane_m1_m3, op_point_empty).run_with_stability_derivatives()
# aero_empty_SM = (aero_empty['x_np'] - empty_mass_props.x_cg) / wing.mean_geometric_chord()
# opti.subject_to([
#     aero_empty_SM > 0.1,
#     aero_empty_SM < 0.3
# ])

prop_rpm_empty = opti.variable(init_guess=18000, lower_bound=0, upper_bound=24000, scale=1000, category="prop_rpm") #1/s
throttle_empty = opti.variable(init_guess=1.0, lower_bound=0, upper_bound=1, scale=0.1, category="throttle", freeze=False)  # Throttle
m3_lap_length = opti.variable(init_guess=1000*units.foot*1.5, scale=10, category="m3_lap_length")  # Lap length

tip_velocity_empty = prop_rpm_empty*(prop_diameter*units.inch*np.pi/60/2)

# ballast_empty = opti.variable(init_guess=0, lower_bound=0, upper_bound=10, scale=1, category="ballast_empty")  # Ballast mass
# ballast_x_empty = opti.variable(init_guess=-0.1, lower_bound=-0.2, upper_bound=0.5, scale=0.1, category="ballast_x_empty")  # Ballast cg
# empty_mass_props = empty_mass_props + asb.MassProperties(mass = ballast_empty, x_cg = ballast_x_empty, y_cg = 0, z_cg = 0)

opti.subject_to([
    thrust_empty == 4.392399e-8*prop_rpm_empty*prop_diameter**3.5/(prop_pitch**0.5)*(4.23333e-4*prop_rpm_empty*prop_pitch-velocity_empty),
    prop_rpm_empty == Kv*throttle_empty*m3_voltage,  
    thrust_empty < Pmax/velocity_empty,
    tip_velocity_empty < 343,
])

drag_empty = aero_empty['D']*2 + fuel_pods(velocity_empty, diameter, num_pods=2) + wheel(velocity_empty, wheel_diameter, CD_landing_gear)
CD_empty = drag_empty/(0.5*rho*velocity_empty**2*S)
opti.subject_to([
    drag_empty == thrust_empty,
    aero_empty['L'] == empty_mass_props.mass*g,
    number_of_laps_empty*m3_lap_length/velocity_empty < 5*60
])

m3_vstall = np.sqrt(2*empty_mass_props.mass*g/(rho*S*1.6))
avg_speed_takeoff_m3 = 0.9*m3_vstall
m3_takeoff_max_thrust = 3.392399e-8*prop_rpm_full*prop_diameter**3.5/(prop_pitch**0.5)*(4.23333e-4*prop_rpm_full*prop_pitch-avg_speed_takeoff_m3)
m3_takeoff_drag = 0.5*rho*avg_speed_takeoff_m3**2*CD_empty*S
m3_takeoff_accel = (m3_takeoff_max_thrust-m3_takeoff_drag)/total_mass_props.mass
m3_takeoff_distance = m3_vstall**2/(2*m3_takeoff_accel)
m3_takeoff_energy = m3_takeoff_distance*m3_takeoff_max_thrust/propulsion_efficiency
m3_cruise_energy = thrust_empty*m3_lap_length*(number_of_laps_empty+1)/propulsion_efficiency
m3_takeoff_time = m3_takeoff_distance/avg_speed_takeoff_m3
m3_total_energy = 4500*m3_voltage*3.6 #Joules, 4500mAh battery
opti.subject_to(
    m3_takeoff_energy + m3_cruise_energy < total_energy
)

m3_turning_velocity = velocity_empty*1.2
m3_turning_radius = m3_turning_velocity**2/(g*np.sqrt((turn_load_factor)**2-1))
m3_distance_for_turn = 2*np.pi*m3_turning_radius*2 #in 1 lap, 2 half turns, + 1 full turn


opti.subject_to(m3_lap_length == downwind_length*2 + m3_distance_for_turn*2)

m3_time = number_of_laps_empty*m3_lap_length/velocity_empty + m3_takeoff_time

opti.subject_to([
    m3_time < 5*60,
                    ])

bonus = 2.5 # assume we get glider bonus, we didn't ;(
glider_mass = 0.249/units.lbm #assume we hit worst-case glider-mass. We were lighter. 
# Athough this is an important point. Changing this value will actually change the design of the main aircraft, 
# based on the relative importance of M2 vs M3. Worth thinking about.
mission3_score = number_of_laps_empty + bonus/glider_mass

CLmax = 1.6 #assumed
# Vstall = 15.7
# Vstall = opti.variable(init_guess=8, lower_bound=2, upper_bound=15, scale=10, category="Vstall")
# Vstall = np.sqrt(total_mass_props.mass*g/(0.5*rho*S*CLmax))
# Vstall = opti.variable(init_guess=15.7, lower_bound=2, upper_bound=25, scale=10, category="Vstall", freeze=True)

""" This year, our optimal aircraft lived in a corner of VERY fast and VERY HEAVY. This made us nervous, because this is hard to test, build, 
but also just generally dangerous in case of failure or loss of control too. When we have a ~55lb plane, we're in the realm of seriously hurting someone, if not death.
So, we somewhat arbitrarily constrained our Vmin speed. """
Vmin = opti.variable(init_guess=25, lower_bound=2, upper_bound=25, scale=10, category="Vmin") 

#ASSUMING CD0 = 0.05
CD0 = 0.04
span_efficiency = 0.85
opti.subject_to([
    Vmin**2 == total_mass_props.mass*g/(.5*rho*b*np.sqrt(CD0*span_efficiency*np.pi*S))
])

# This section is completely irrelevant, I have another repo that makes really pretty V-n diagrams :P.
terminal_velocity = opti.variable(init_guess=80, lower_bound=2, upper_bound=150, scale=10, category="terminal_velocity")
# alpha_terminal = opti.variable(init_guess=0, lower_bound=-10, upper_bound=10, scale=5, category="alpha_terminal")  # Angle of attack
op_point_terminal = asb.OperatingPoint(velocity=terminal_velocity, alpha=0)
aero_terminal = asb.AeroBuildup(plane_m2, op_point=op_point_terminal).run()
terminal_drag = aero_terminal['D']*2 + fuel_pods(terminal_velocity, diameter, num_pods=2) + wheel(terminal_velocity, wheel_diameter, CD_landing_gear)
CD_terminal = terminal_drag/(0.5*rho*terminal_velocity**2*S)

opti.subject_to([
    terminal_velocity**2 == total_mass_props.mass*g/(0.5*rho*S*CD_terminal),
])


"""Now we get to the important part. 
DBF works based on normalized scores. So, we first optimize M2, then we optimize M3, then we optimize total_score, defined below.
This can all be optimized in a single run, however, the whole point of this script is to see how the optimal design changes as we subtly change the problem. 
Therefore, execution speed is very important, and simply, this is much faster."""

mission2_opt = 0.0855
mission3_opt = 12.2462
total_score = mission2_score/mission2_opt + mission3_score/mission3_opt
opti.minimize(-(total_score)) # minimize negative, so maximize

sol = opti.solve(behavior_on_failure='return_last')

#print results
# geometry output
print('\n\n__Geometry__:')
print("Wing:")
print(f"Root chord: {round(sol(rc), 2)} [m]")
print(f"Tip chord: {round(sol(tc), 2)} [m]")
print(f"Taper ratio: {round(sol(taper), 2)} [-]")
print(f"Span: {round(sol(b), 2)} [m] limit: {6*units.foot} [m])")
print(f"AR: {round(sol(AR), 2)} [-]")
print(f"S: {round(sol(S), 2)}")
print(f"Ultimate load factor (positive): {round(sol(positive_limit_load_factor), 2)}")
print(f"Ultimate load factor (negative): {round(sol(negative_limit_load_factor), 2)}")
# print(f"Sweep: {round(sol(np.degrees(sweep)), 2)} [deg]")
print(f'Motor x: {round(sol(motor_x), 2)} [m]')
print(f'Battery x: {round(sol(battery_x), 2)} [m]')
print(f'ESC x: {round(sol(esc_x), 2)} [m]')
print(f'Payload cg: {round(sol(payload_cg), 2)} [m]')
print(f'Pylon y: {round(sol(pylon_y), 2)} [m] (limit: {pylon_y_limit} [m])')
print(f'Boom deflection: {round(sol(boom_deflection), 3)} [m] (should be under {round(tail_deflection_limit, 3)} [m])')
print(f'Boom inner diameter: {round(sol(boom_inner_h), 4)} [m]')
print(f'Boom outer diameter: {round(sol(boom_outer_h), 4)} [m]')
print(f'Boom length: {round(sol(boom_length), 2)} [m]')
print(f'Boom torsional shear stress: {round(sol(torsional_shear_stress_boom), 2)} [Pa] (should be under {material_max_shear_stress})')
# print(f'Spar outer diameter: {round(sol(spar_outer_h), 4)} [m]')
# print(f'Spar inner diameter: {round(sol(spar_inner_h), 4)} [m]')
print('_____________________')
print('Spar values:')
print(f'Spar cap thickness: {round(sol(spar_cap_thickness), 4)} [m]')
print(f'Spar core width: {round(sol(spar_width), 4)} [m]')
print(f'Positive wing deflection: {round(sol(deflection_positive), 3)} [m] (should be under {round(tip_deflection_limit, 3)} [m])')
print(f'Negative wing deflection: {round(sol(deflection_negative), 3)} [m] (should be under {round(tip_deflection_limit, 3)} [m])')
print(f'Shear stress (positive): {round(abs(sol(shear_stress_core_positive)), 2)} [Pa] (should be under {core_shear_strength})')
print(f'Shear stress (negative): {round(abs(sol(shear_stress_core_negative)), 2)} [Pa] (should be under {core_shear_strength})')
print(f'Bending stress (positive): {round(abs(sol(bending_stress_positive)), 2)} [Pa] (should be under {cap_material_yield_strength})')
print(f'Bending stress (negative): {round(abs(sol(bending_stress_negative)), 2)} [Pa] (should be under {cap_material_yield_strength})')
print("\n____________________")

# tail output
print("\nTail:")
print(f"Tail distance: {round(sol(tail_dis), 2)}")
print(f"Vertical tail span: {round(sol(v_b), 2)}")
print(f"Vertical tail chord: {round(sol(v_c), 2)}")
print(f"Horizontal tail span: {round(sol(h_b), 2)}")
print(f"Horizontal tail V: {round(sol(H_tail_volume), 2)} (should be 0.3 < Vh < 0.6)")
print(f"Vertical Tail V: {round(sol(V_tail_volume), 2)} (should be 0.02 < Vv < 0.05)")
print("\n____________________")

# propulsion
print("\n__Propulsion__:")
print(f"Current: {round(sol(I), 2)} [A]")
print(f"KV: {round(sol(Kv), 2)} [rpm/V]")
print(f"Power(max): {round(sol(Pmax), 2)} [W]")
print(f'Pitch: {round(sol(prop_pitch), 2)} [inch?]')
print(f"Prop diameter: {round(sol(prop_diameter), 2)} [inch]")
print(f"Propulsion efficiency: {round(sol(propulsion_efficiency), 2)} [-]")
print(f'RPM max: {round(sol(rpm_max), 2)} [rpm]')
print(f'Max disk loading: {round(sol(disk_loading), 2)} [N/m^2]')
print("\n____________________")

# objective
print("\n__Objective__:")
print(f'Total score: {round(sol(total_score), 2)}')
print("\n____________________\n")
print(f'Mission 2: {round(sol(mission2_score), 4)} Percent of estimated optimum {round(sol(mission2_score)/mission2_opt*100, 2)}%')
print(f'Velocity: {round(sol(velocity_full), 2)} [m/s]')
print(f'Alpha: {round(sol(alpha_full), 2)} [deg]')
print(f'CL: {round(sol(aero_full["CL"]), 2)}')
print(f'CD: {round(sol(CD_full), 2)}')
print(f'Reynolds: {round(sol(op_point_full.reynolds(rc)), 2)}')
print(f'Thrust: {round(sol(thrust_full), 2)} [N]')
print(f'Throttle: {round(sol(throttle_full), 2)}')
print(f'RPM: {round(sol(prop_rpm_full), 2)}')
# print(f'SM: {round(sol(SM_full), 2)}')
print(f'Turn radius: {round(sol(m2_turning_radius), 2)} [m]')
print(f"Total mass: {round(sol(total_mass_props.mass), 2)} [kg]")
print(f"Payload mass: {round(sol(payload_mass_props.mass), 2)} [kg]")
print(f"Empty mass: {round(sol(empty_mass_props.mass), 2)} [kg]")
print(f"Flight time: {round(sol(m2_time), 2)} [sec]")
print(f"Flight time per lap: {round(sol(m2_time)/3, 2)} [sec]")
print(f"Pod Volume: {round(sol(total_fuel_volume), 2)} [m^3]")
print(f"Required fuel density: {round(sol(payload_mass)/sol(total_fuel_volume), 2)} [kg/m^3]")
print(f'Energy used on takeoff: {round(sol(m2_takeoff_energy), 2)} [J] {round(sol(m2_takeoff_energy)/total_energy*100, 2)}%')
print(f'Energy used in cruise: {round(sol(m2_cruise_energy), 2)} [J] {round(sol(m2_cruise_energy)/total_energy*100, 2)}%')
print(f'Takeoff distance: {round(sol(m2_takeoff_distance), 2)} [m]')
# print("Ballast: ", round(sol(ballast_full), 2))
# print("Ballast x: ", round(sol(ballast_x_full), 2))
print(f'Cg location: {round(sol(total_mass_props.x_cg), 2)}')
print(f'Neutral point: {round(sol(aero_full["x_np"]), 2)}')
print("\n____________________\n")
print(f'Mission 3: {round(sol(mission3_score), 4)} Percent of estimated optimum {round(sol(mission3_score)/mission3_opt*100, 2)}%')
print(f"Total laps: {round(sol(number_of_laps_empty), 2)}")
print(f"Flight time: {round(sol(m3_time), 2)} [sec]")
print(f"Flight time per lap: {round(sol(m3_time)/sol(number_of_laps_empty), 2)} [sec]")
print(f"Mass: {round(sol(empty_mass_props.mass), 2)} [kg]")
print(f"Velocity: {round(sol(velocity_empty), 2)} [m/s]")
print(f"Alpha: {round(sol(alpha_empty), 2)} [deg]")
print(f"CL: {round(sol(aero_empty['CL']), 2)}")
print(f"CD: {round(sol(aero_empty['CD']), 2)}")
print(f"Reynolds: {round(sol(op_point_empty.reynolds(rc)), 2)}")
print(f"Thrust: {round(sol(thrust_empty), 2)} [N]")
print(f'Throttle: {round(sol(throttle_empty), 2)}')
print(f'RPM: {round(sol(prop_rpm_empty), 2)}')
# print(f'SM: {round(sol(aero_empty_SM), 2)}')
print(f"Turn radius: {round(sol(m3_turning_radius), 2)} [m]")
print(f"Energy used on takeoff: {round(sol(m3_takeoff_energy), 2)} [J] {round(sol(m3_takeoff_energy)/total_energy*100, 2)}%")
print(f"Energy used in cruise: {round(sol(m3_cruise_energy), 2)} [J] {round(sol(m3_cruise_energy)/total_energy*100, 2)}%")
print(f'Takeoff distance: {round(sol(m3_takeoff_distance), 2)} [m]')
# print("Ballast: ", round(sol(ballast_empty), 2))
# print("Ballast x: ", round(sol(ballast_x_empty), 2))
print(f'Cg location: {round(sol(empty_mass_props.x_cg), 2)}')
print(f'Neutral point: {round(sol(aero_empty["x_np"]), 2)}')

print("\n____________________\n")
print('\n\n__Mass Breakdown__:')
empty_mass_props = sol(empty_mass_props)
print('Everything is divided by total empty mass')
print(f"Wing mass: {round(sol(wing_mass_props.mass), 2)} [kg], {round(sol(wing_mass_props.mass)/sol(empty_mass_props.mass)*100, 2)} [%]")
print(f"    Rib mass: {round(sol(wing_rib_mass_props.mass), 2)} [kg], {round(sol(wing_rib_mass_props.mass)/sol(wing_mass_props.mass)*100, 2)} [% of wing]")
print(f"    Spar mass: {round(sol(wing_spar_mass_props.mass), 2)} [kg], {round(sol(wing_spar_mass_props.mass)/sol(wing_mass_props.mass)*100, 2)} [% of wing]")
print(f"         Cap mass: {round(sol(spar_carbon_massprops_epoxy.mass), 2)} [kg], {round(sol(spar_carbon_massprops_epoxy.mass)/sol(wing_spar_mass_props.mass)*100, 2)} [% of spar]")
print(f"              Cap epoxy: {round(sol(spar_carbon_massprops_epoxy.mass)-sol(spar_carbon_massprops_no_epoxy.mass), 2)} [kg],")
print(f"         Fiberglass mass: {round(sol(spar_fiberglass_massprops_epoxy.mass), 2)} [kg], {round(sol(spar_fiberglass_massprops_epoxy.mass)/sol(wing_spar_mass_props.mass)*100, 2)} [% of spar]")
print(f"              Fiberglass epoxy: {round(sol(spar_fiberglass_massprops_epoxy.mass)-sol(spar_fiberglass_massprops_no_epoxy.mass), 2)} [kg],")
print(f"         Core mass: {round(sol(spar_core_massprops.mass), 2)} [kg], {round(sol(spar_core_massprops.mass)/sol(wing_spar_mass_props.mass)*100, 2)} [% of spar]")
print(f"         Total spar epoxy mass: {round(sol(spar_epoxy_mass), 2)} [kg], {round(sol(spar_epoxy_mass)/sol(wing_mass_props.mass)*100, 2)} [% of wing] (includes epoxy)")
print(f"    Monokote mass: {round(sol(wing_monokote_mass_props.mass), 2)} [kg], {round(sol(wing_monokote_mass_props.mass)/sol(wing_mass_props.mass)*100, 2)} [% of wing]")
print(f"    Carbon leading edge mass: {round(sol(wing_carbon_leading_edge_mass_props.mass), 2)} [kg], {round(sol(wing_carbon_leading_edge_mass_props.mass)/sol(wing_mass_props.mass)*100, 2)} [% of wing]")
print(f"    Flap mass: {round(sol(flap_mass_props.mass), 2)} [kg], {round(sol(flap_mass_props.mass)/sol(wing_mass_props.mass)*100, 2)} [% of wing]")
print(f"    Aileron mass: {round(sol(aileron_mass_props.mass), 2)} [kg], {round(sol(aileron_mass_props.mass)/sol(wing_mass_props.mass)*100, 2)} [% of wing]")
print(f"Vertical tail mass: {round(sol(vt_mass_props.mass), 2)} [kg], {round(sol(vt_mass_props.mass)/sol(empty_mass_props.mass)*100, 2)} [%]")
print(f"    Foam mass: {round(sol(vt_foam_massprops.mass), 2)} [kg], {round(sol(vt_foam_massprops.mass)/sol(vt_mass_props.mass)*100, 2)} [% of vertical tail]")
print(f"    Fiberglass mass: {round(sol(vt_fiberglass_massprops.mass), 2)} [kg], {round(sol(vt_fiberglass_massprops.mass)/sol(vt_mass_props.mass)*100, 2)} [% of vertical tail]")
print(f"    Carbon mass: {round(sol(vt_carbon_massprops.mass), 2)} [kg], {round(sol(vt_carbon_massprops.mass)/sol(vt_mass_props.mass)*100, 2)} [% of vertical tail]")
# print(f"    Rib mass: {round(sol(vt_rib_mass_props.mass), 2)} [kg], {round(sol(vt_rib_mass_props.mass)/sol(vt_mass_props.mass)*100, 2)} [% of vertical tail]")
# print(f"    Spar mass: {round(sol(vt_spar_mass_props_tail.mass), 2)} [kg], {round(sol(vt_spar_mass_props_tail.mass)/sol(vt_mass_props.mass)*100, 2)} [% of vertical tail]")
# print(f"    Monokote mass: {round(sol(vt_monokote_mass_props.mass), 2)} [kg], {round(sol(vt_monokote_mass_props.mass)/sol(vt_mass_props.mass)*100, 2)} [% of vertical tail]")
print(f"Horizontal tail mass: {round(sol(ht_mass_props.mass), 2)} [kg], {round(sol(ht_mass_props.mass)/sol(empty_mass_props.mass)*100, 2)} [%] (includes epoxy)")
print(f"    Foam mass: {round(sol(ht_foam_massprops.mass), 2)} [kg], {round(sol(ht_foam_massprops.mass)/sol(ht_mass_props.mass)*100, 2)} [% of horizontal tail]")
print(f"    Fiberglass mass: {round(sol(ht_fiberglass_massprops.mass), 2)} [kg], {round(sol(ht_fiberglass_massprops.mass)/sol(ht_mass_props.mass)*100, 2)} [% of horizontal tail]")
print(f"    Carbon mass: {round(sol(ht_carbon_massprops.mass), 2)} [kg], {round(sol(ht_carbon_massprops.mass)/sol(ht_mass_props.mass)*100, 2)} [% of vertical tail]")
# print(f"    Rib mass: {round(sol(ht_rib_mass_props.mass), 2)} [kg], {round(sol(ht_rib_mass_props.mass)/sol(horizontal_tail_mass_props.mass)*100, 2)} [% of horizontal tail]")
# print(f"    Spar mass: {round(sol(ht_spar_mass_props_tail.mass), 2)} [kg], {round(sol(ht_spar_mass_props_tail.mass)/sol(horizontal_tail_mass_props.mass)*100, 2)} [% of horizontal tail]")
# print(f"    Monokote mass: {round(sol(ht_monokote_mass_props.mass), 2)} [kg], {round(sol(ht_monokote_mass_props.mass)/sol(horizontal_tail_mass_props.mass)*100, 2)} [% of horizontal tail]")
print(f"Landing Gear mass: {round(sol(landing_gear_mass_props.mass), 2)} [kg], {round(sol(landing_gear_mass_props.mass)/sol(empty_mass_props.mass)*100, 2)} [%]")
print(f"Pylon mass: {round(sol(pylon_mass_props.mass), 2)} [kg], {round(sol(pylon_mass_props.mass)/sol(empty_mass_props.mass)*100, 2)} [%]")
print(f"Boom mass: {round(sol(boom_mass), 2)} [kg], {round(sol(boom_mass)/sol(empty_mass_props.mass)*100, 2)} [%]")
print(f"Electronics mass: {round(sol(electronics_mass_props.mass), 2)} [kg], {round(sol(electronics_mass_props.mass)/sol(empty_mass_props.mass)*100, 2)} [%]")
print(f"    Motor mass: {round(sol(motor_mass), 2)} [kg], {round(sol(motor_mass)/sol(electronics_mass_props.mass)*100, 2)} [% of electronics]")
print(f"    Battery mass: {round(sol(battery_mass), 2)} [kg], {round(sol(battery_mass)/sol(electronics_mass_props.mass)*100, 2)} [% of electronics]")
print(f"    ESC mass: {round(sol(esc_mass), 2)} [kg], {round(sol(esc_mass)/sol(electronics_mass_props.mass)*100, 2)} [% of electronics]")
print(f"    Wing servo mass: {round(sol(wing_servo_massprops.mass), 2)} [kg], {round(sol(wing_servo_massprops.mass)/sol(electronics_mass_props.mass)*100, 2)} [% of electronics]")
print(f"    Tail servo mass: {round(sol(tail_servo_massprops.mass), 2)} [kg], {round(sol(tail_servo_massprops.mass)/sol(electronics_mass_props.mass)*100, 2)} [% of electronics]")
print(f"    Servo wires (and other): {round(sol(wire_massprops.mass), 2)} [kg], {round(sol(wire_massprops.mass)/sol(electronics_mass_props.mass)*100, 2)} [% of electronics]")
print(f"Payload mass: {round(sol(payload_mass_props.mass), 4)} [kg], {round(sol(payload_mass)/sol(total_mass_props.mass)*100, 2)} [%]")
print(f"Total mass: {round(sol(total_mass_props.mass), 2)} [kg]")
print(f"Payload mass fraction: {round(sol(payload_mass_props.mass)/sol(total_mass_props.mass)*100, 2)} [%]")

print("\n____________________\n")
print(f'Terminal velocity: {round(sol(terminal_velocity), 2)} [m/s]')


plane_m2 = sol(plane_m2)    
import matplotlib.pyplot as plt
# plane_m2.draw_three_view()
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

# Draw the airplane wireframe with axis hidden
ax = plane_m2.draw_wireframe(ax=ax, set_axis_visibility=False)

# Show the modified plot
plt.show()
# asb.Airplane().draw()
